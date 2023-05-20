################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Anemometer_G473CBT7_MATLAB
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Anemometer_G473CBT7_MATLAB.elf \

MAP_FILES += \
Anemometer_G473CBT7_MATLAB.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Anemometer_G473CBT7_MATLAB.list \


# All Target
all: main-build

# Main-build Target
main-build: Anemometer_G473CBT7_MATLAB.elf secondary-outputs

# Tool invocations
Anemometer_G473CBT7_MATLAB.elf Anemometer_G473CBT7_MATLAB.map: $(OBJS) $(USER_OBJS) /home/ed/work/gitprojects/Anemometer/stm32cubeide/Anemometer_G474CBT6_MATLAB/STM32G473CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Anemometer_G473CBT7_MATLAB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/ed/work/gitprojects/Anemometer/stm32cubeide/Anemometer_G474CBT6_MATLAB/STM32G473CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Anemometer_G473CBT7_MATLAB.map" -Wl,--gc-sections -static -LARM_MATH_CM4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Anemometer_G473CBT7_MATLAB.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Anemometer_G473CBT7_MATLAB.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Anemometer_G473CBT7_MATLAB.elf Anemometer_G473CBT7_MATLAB.list Anemometer_G473CBT7_MATLAB.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
