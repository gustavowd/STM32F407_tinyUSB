################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/portable/st/synopsys/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/portable/st/stm32_fsdev/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/host/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/device/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/common/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/video/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/vendor/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/usbtmc/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/net/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/msc/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/midi/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/hid/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/dfu/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/cdc/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/bth/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/class/audio/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/src/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/hw/bsp/stm32f4/subdir.mk
-include Middlewares/Third_Party/tinyusb-0.15.0/hw/bsp/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := STM32F407_tinyUSB
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F407_tinyUSB.elf \

MAP_FILES += \
STM32F407_tinyUSB.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F407_tinyUSB.list \


# All Target
all: main-build

# Main-build Target
main-build: STM32F407_tinyUSB.elf secondary-outputs

# Tool invocations
STM32F407_tinyUSB.elf STM32F407_tinyUSB.map: $(OBJS) $(USER_OBJS) /home/gustavo/STM32CubeIDE/workspace_1.10.1/STM32F407_tinyUSB/STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STM32F407_tinyUSB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/gustavo/STM32CubeIDE/workspace_1.10.1/STM32F407_tinyUSB/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32F407_tinyUSB.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F407_tinyUSB.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F407_tinyUSB.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STM32F407_tinyUSB.elf STM32F407_tinyUSB.list STM32F407_tinyUSB.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
