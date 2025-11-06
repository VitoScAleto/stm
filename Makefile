# Project name
PROJECT = stm32f103_project

# Toolchain
CC = C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin/arm-none-eabi-gcc.exe
OBJCOPY = C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin/arm-none-eabi-objcopy.exe
SIZE = C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin/arm-none-eabi-size.exe
OPENOCD = C:/Users/pushk/Desktop/Stm_SWD/OpenOCD-20250710-0.12.0/bin/openocd.exe
MKDIR = powershell -Command "New-Item -ItemType Directory -Force -Path"

# MCU settings
MCU = cortex-m3
TARGET = STM32F103x6

# Directories
SRC_DIR = src
INC_DIR = inc
CMSIS_DIR = CMSIS
LD_DIR = ld
BUILD_DIR = build

# Source files
SRCS = $(SRC_DIR)/main.c $(SRC_DIR)/system_stm32f1xx.c $(SRC_DIR)/startup_stm32f103x6.s

# Compiler flags
CFLAGS = -mcpu=$(MCU) -mthumb
CFLAGS += -D$(TARGET) -Og -g3
CFLAGS += -Wall -Wextra
CFLAGS += -I$(INC_DIR) -I$(CMSIS_DIR)/Include
CFLAGS += -ffunction-sections -fdata-sections

# Linker flags
LDFLAGS = -T$(LD_DIR)/STM32F103X6_FLASH.ld
LDFLAGS += -specs=nosys.specs -specs=nano.specs
LDFLAGS += -Wl,--gc-sections -Wl,--print-memory-usage

# Objects
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
OBJS := $(OBJS:$(SRC_DIR)/%.s=$(BUILD_DIR)/%.o)

# Rules
all: $(BUILD_DIR)/$(PROJECT).elf

$(BUILD_DIR)/$(PROJECT).elf: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)
	$(SIZE) $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	$(MKDIR) $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.s
	$(MKDIR) $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD_DIR)/$(PROJECT).elf
	$(OPENOCD) -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program $< verify reset exit"

clean:
	powershell -Command "Remove-Item -Recurse -Force $(BUILD_DIR)"

.PHONY: all flash clean