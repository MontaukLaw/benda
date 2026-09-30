################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# $ Copyright 2018-2022 Cypress Semiconductor Apache2 $
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED
# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make library-manager' from command line), which will also update Eclipse IDE launch
# configurations.
TARGET=APP_CY8CPROTO-041TP

# Select the hardware/identity variant at build time:
#   make build FINGER_TYPE=INDEX_FINGER
#   make build FINGER_TYPE=THUMB
#   make build FINGER_TYPE=PINKY
FINGER_TYPE?=INDEX_FINGER

ifeq ($(FINGER_TYPE),INDEX_FINGER)
  FINGER_TYPE_DEFINE=1
else ifeq ($(FINGER_TYPE),THUMB)
  FINGER_TYPE_DEFINE=2
else ifeq ($(FINGER_TYPE),PINKY)
  FINGER_TYPE_DEFINE=3
else
  $(error FINGER_TYPE must be INDEX_FINGER, THUMB, or PINKY.)
endif

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=mtb-example-psoc4-msclp-finger-$(FINGER_TYPE)

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox software
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
#
# If CONFIG is manually edited, ensure to update or regenerate launch configurations
# for your IDE.
CONFIG=Debug

# Build one application communication variant at a time. Both variants produce
# a signed cyacd2 for UART DFU and an iap.bin for the A5/CRC8 I2C protocol.
TRANSPORT_OPT?=UART
CY_BUILD_LOCATION=./build/$(FINGER_TYPE)/$(TRANSPORT_OPT)
# 排除本地分析产物和Python临时依赖，避免自动发现把它们当作固件源码。
CY_IGNORE=./build ./tmp ./analysis_outputs ./firmware_variants/thumb_pinky/config_sources

# The current INDEX_FINGER BSP is a 20-point matrix. Keep the legacy algorithm
# sources available, but do not compile or run them in the temporary raw mode.
ifeq ($(FINGER_TYPE),INDEX_FINGER)
  RAW_ONLY?=0
else
  RAW_ONLY?=0
endif
ifeq ($(RAW_ONLY),1)
  ifneq ($(FINGER_TYPE),INDEX_FINGER)
    $(error RAW_ONLY=1 requires the INDEX_FINGER 20-point BSP.)
  endif
  ifneq ($(TRANSPORT_OPT),UART)
    $(error RAW_ONLY=1 supports UART only.)
  endif
  APPNAME=mtb-example-psoc4-raw20
  CY_IGNORE+=./main.c ./tuner.c ./app_iic.c ./app_flash.c \
    ./baseline_tracker.c ./channel_state_machine.c ./sensor_algorithm.c \
    ./process_channel_sample.c ./sensor_bus_slave.c ./force_vector_cal.c \
    ./low_force_calibration.c ./die_temperature.c \
    ./capsense_affine_calibration.c ./capsense_autorange.c \
    ./capsense_channel_config.c ./capsense_equivalent.c \
    ./capsense_nsub_control.c ./capsense_runtime.c
else ifeq ($(RAW_ONLY),0)
  CY_IGNORE+=./raw_stream.c
else
  $(error RAW_ONLY must be 0 or 1.)
endif

# THUMB/PINKY share the five-independent-button CapSense topology imported
# from THUM_PINKY_NEW. Generated sources cannot be switched by the C
# preprocessor because both variants export the same cycfg symbols, so the
# make-time selector excludes the other generated source set during discovery.
ifeq ($(FINGER_TYPE),INDEX_FINGER)
  CY_IGNORE+=./firmware_variants/thumb_pinky/generated
else
  CY_IGNORE+=./bsps/TARGET_APP_CY8CPROTO-041TP/config/GeneratedSource
endif

# The ModusToolbox post-build runs in its bundled bash, whose PATH does not
# necessarily contain the Windows Python installation.
ifeq ($(OS),Windows_NT)
  # PYTHON?=C:/Windows/py.exe -3.12
  PYTHON?=py -3
else
  PYTHON?=python3
endif

# If set to "true" or "1", display full command-lines when building.
VERBOSE=


################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS=

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=

ifneq ($(FINGER_TYPE),INDEX_FINGER)
  INCLUDES+=firmware_variants/thumb_pinky/generated
endif

# Add additional defines to the build process (without a leading -D).
DEFINES=CY_APP_ID=1 DEVICE_FINGER_TYPE=$(FINGER_TYPE_DEFINE)

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS=

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
# Keep the storage boundary in a tracked linker script, not in ignored BSP files.
LINKER_SCRIPT=linker/dfu_cm0p_app1.ld

# Custom pre-build commands to run.
PREBUILD=$(PYTHON) tools/check_flash_layout.py

# Custom post-build commands to run.

ifeq ($(TRANSPORT_OPT), UART)
  DEFINES+=TUNER_INTERFACE_TYPE=1 APP_UART_DFU=1
else ifeq ($(TRANSPORT_OPT), I2C)
  DEFINES+=TUNER_INTERFACE_TYPE=3 APP_I2C_IAP=1
else
  $(error TRANSPORT_OPT must be UART or I2C.)
endif

POSTBUILD+=$(CY_TOOL_cymcuelftool_EXE_ABS) --sign $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).elf CRC --output $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME)_crc.elf && \
           $(CY_TOOL_cymcuelftool_EXE_ABS) -P $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME)_crc.elf --output $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME)_crc.cyacd2 && \
           $(PYTHON) tools/package_iap.py $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).hex $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME).iap.bin

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# software provided compiler by default).
# CY_COMPILER_PATH=


# Locate ModusToolbox helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox software in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder). Make sure you use forward slashes.
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

include $(CY_TOOLS_DIR)/make/start.mk
