################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Time\ Base\ Interrupt.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Time\ Base\ Interrupt.list \

OBJCOPY_BIN += \
Time\ Base\ Interrupt.bin \


# All Target
all: Time\ Base\ Interrupt.elf secondary-outputs

# Tool invocations
Time\ Base\ Interrupt.elf: $(OBJS) $(USER_OBJS) C:\Users\dveer\STM32CubeIDE\workspace_1.3.0\Time\ Base\ Interrupt\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "Time Base Interrupt.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\dveer\STM32CubeIDE\workspace_1.3.0\Time Base Interrupt\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Time Base Interrupt.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Time\ Base\ Interrupt.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Time Base Interrupt.list"
	@echo 'Finished building: $@'
	@echo ' '

Time\ Base\ Interrupt.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Time Base Interrupt.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
