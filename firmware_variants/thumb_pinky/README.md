# THUMB / PINKY CapSense配置

本目录保存从 `D:\work\INFENEON_MAIN_LINE\THUM_PINKY_NEW` 引入的5个独立CSX Button配置。

- `config_sources/design.modus`：引脚、时钟和外设源配置。
- `config_sources/design.cycapsense`：5通道CapSense源配置。
- `generated/`：与上述配置配套的Configurator生成源码；不要手工修改。

`THUMB`和`PINKY`当前共用这套硬件配置，设备名称由根目录`device_variant.h`区分。详细映射和构建命令见[固件变体](../../FIRMWARE_VARIANTS_CN.md)，协议与Flash边界见[当前实现](../../CURRENT_SYSTEM_CN.md)。食指启动RefCDAC=30不代表本目录生成配置也改为30。

`config_sources/`被构建系统排除，仅作为Configurator源文件保存，避免和食指配置同时被自动发现。修改后应重新生成，并用完整的新生成结果替换`generated/`，随后分别编译THUMB和PINKY验证。
