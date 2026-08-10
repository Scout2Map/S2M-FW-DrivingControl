# Scout2Map UGV - STM32F103C8T6 drive controller
# Bare metal build, no HAL, CMSIS headers only

TARGET  = scout2map_drive
BUILD   = build

# CMSIS device and core headers, see README for how to fetch them
CMSIS_DIR = cmsis

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Every .c under app/ and lib/<layer>/ is picked up automatically
# Adding a new module means dropping the file in, no Makefile edit
SRCS = $(wildcard app/*.c) $(wildcard lib/*/*.c)

# Headers live next to their .c file, so include every source directory
INC_DIRS  = $(sort $(dir $(SRCS)))
INCS      = $(addprefix -I,$(INC_DIRS))
INCS     += -Iconfig
INCS     += -I$(CMSIS_DIR)/Include
INCS     += -I$(CMSIS_DIR)/Device/ST/STM32F1xx/Include

DEFS = -DSTM32F103xB

CFLAGS  = -mcpu=cortex-m3 -mthumb -std=c11
CFLAGS += -Og -g3 -Wall -Wextra -Wshadow
CFLAGS += -ffunction-sections -fdata-sections -fno-common
CFLAGS += $(INCS) $(DEFS)

LDFLAGS  = -mcpu=cortex-m3 -mthumb
LDFLAGS += -Tld/stm32f103c8.ld
LDFLAGS += -nostartfiles -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map
LDFLAGS += --specs=nosys.specs -lm

# Mirror the source tree inside build/ so files with the same name
# in different layers never collide
OBJS = $(addprefix $(BUILD)/, $(SRCS:.c=.o))
DEPS = $(OBJS:.o=.d)

all: $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# ST-LINK V2 clone is happier with OpenOCD than with CubeProgrammer
flash: $(BUILD)/$(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	  -c "program $(BUILD)/$(TARGET).elf verify reset exit"

# Host build of the hardware independent layer, no board required
# Lets the control logic be unit tested before the hardware is ready
test:
	$(MAKE) -C test

clean:
	rm -rf $(BUILD)

.PHONY: all flash test clean

-include $(DEPS)
