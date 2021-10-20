################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Firmware_Test_LV_BMS.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Firmware_Test_LV_BMS.list \

OBJCOPY_BIN += \
Firmware_Test_LV_BMS.bin \


# All Target
all: Firmware_Test_LV_BMS.elf secondary-outputs

# Tool invocations
Firmware_Test_LV_BMS.elf: $(OBJS) $(USER_OBJS) C:\Users\Juppy\Documents\GitHub\LV-BMS\Firmware_Test\Firmware_Test_LV_BMS\STM32L432KBUX_FLASH.ld
	arm-none-eabi-gcc -o "Firmware_Test_LV_BMS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Juppy\Documents\GitHub\LV-BMS\Firmware_Test\Firmware_Test_LV_BMS\STM32L432KBUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Firmware_Test_LV_BMS.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Firmware_Test_LV_BMS.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Firmware_Test_LV_BMS.list"
	@echo 'Finished building: $@'
	@echo ' '

Firmware_Test_LV_BMS.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Firmware_Test_LV_BMS.bin"
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
