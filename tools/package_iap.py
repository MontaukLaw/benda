"""Package an application Intel HEX file for the custom I2C IAP protocol."""

from __future__ import annotations

import argparse
import binascii
import json
from pathlib import Path
from check_flash_layout import layout_value, check_layout

APP_START = 0x00003600
APP_END = layout_value('APP_FLASH_PARAMETER_START')
APP_SIGNATURE_ADDRESS = APP_END - 4


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crc32c(data: bytes) -> int:
    table = (
        0x00000000,
        0x105EC76F,
        0x20BD8EDE,
        0x30E349B1,
        0x417B1DBC,
        0x5125DAD3,
        0x61C69362,
        0x7198540D,
        0x82F63B78,
        0x92A8FC17,
        0xA24BB5A6,
        0xB21572C9,
        0xC38D26C4,
        0xD3D3E1AB,
        0xE330A81A,
        0xF36E6F75,
    )
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        crc = (crc >> 4) ^ table[crc & 0x0F]
        crc = (crc >> 4) ^ table[crc & 0x0F]
    return (~crc) & 0xFFFFFFFF


def parse_intel_hex(path: Path) -> dict[int, int]:
    memory: dict[int, int] = {}
    base = 0

    for line_number, raw_line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"line {line_number}: missing Intel HEX prefix")

        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError(f"line {line_number}: invalid record length")
        if sum(record) & 0xFF:
            raise ValueError(f"line {line_number}: checksum mismatch")

        size = record[0]
        offset = int.from_bytes(record[1:3], "big")
        record_type = record[3]
        data = record[4 : 4 + size]

        if record_type == 0x00:
            address = base + offset
            for index, value in enumerate(data):
                memory[address + index] = value
        elif record_type == 0x01:
            break
        elif record_type == 0x02:
            base = int.from_bytes(data, "big") << 4
        elif record_type == 0x04:
            base = int.from_bytes(data, "big") << 16

    return memory


def build_image(memory: dict[int, int]) -> bytes:
    check_layout()
    # 不允许将包含Bootloader或参数/DFU行的文件悄悄裁剪后发布。
    if any(0 <= address < 0x20000 and not APP_START <= address < APP_END for address in memory):
        raise ValueError('Application HEX contains bytes outside the protected application range')
    application_bytes = {
        address: value
        for address, value in memory.items()
        if APP_START <= address < APP_END
    }
    if APP_START not in application_bytes:
        raise ValueError(f"application does not start at 0x{APP_START:08X}")

    image = bytearray([0xFF] * (APP_END - APP_START))
    for address, value in application_bytes.items():
        image[address - APP_START] = value

    signature_offset = APP_SIGNATURE_ADDRESS - APP_START
    signature = crc32c(image[:signature_offset])
    image[signature_offset:] = signature.to_bytes(4, "little")
    return bytes(image)


def package(hex_path: Path, output_path: Path) -> dict[str, int | str]:
    image = build_image(parse_intel_hex(hex_path))
    output_path.write_bytes(image)

    metadata: dict[str, int | str] = {
        "file": output_path.name,
        "app_start": APP_START,
        "image_size": len(image),
        "image_crc32": binascii.crc32(image) & 0xFFFFFFFF,
        "row_size": 256,
    }
    output_path.with_suffix(output_path.suffix + ".json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("hex_file", type=Path)
    parser.add_argument("output_file", type=Path)
    args = parser.parse_args()

    metadata = package(args.hex_file, args.output_file)
    print(
        f"IAP image: {metadata['image_size']} bytes, "
        f"CRC32=0x{metadata['image_crc32']:08X}"
    )


if __name__ == "__main__":
    main()
