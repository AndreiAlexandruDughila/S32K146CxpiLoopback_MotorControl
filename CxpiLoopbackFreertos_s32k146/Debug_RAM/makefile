################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include board/subdir.mk
-include SDK/rtos/FreeRTOS/Source/portable/MemMang/subdir.mk
-include SDK/rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include SDK/rtos/FreeRTOS/Source/subdir.mk
-include SDK/platform/drivers/src/pins/subdir.mk
-include SDK/platform/drivers/src/interrupt/subdir.mk
-include SDK/platform/drivers/src/clock/S32K1xx/subdir.mk
-include SDK/platform/devices/S32K146/startup/subdir.mk
-include SDK/platform/devices/subdir.mk
-include Project_Settings/Startup_Code/subdir.mk
-include Project_Settings/Linker_Files/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
CxpiLoopbackFreertos_s32k146.siz \


# All Target
all: CxpiLoopbackFreertos_s32k146.elf secondary-outputs

# Tool invocations
CxpiLoopbackFreertos_s32k146.elf: $(OBJS) C:\NXP\S32DS.3.5\S32DS\software\S32SDK_S32K1XX_RTM_4.0.1\platform\devices\S32K146\linker\gcc\S32K146_128_ram.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Standard S32DS C Linker'
	arm-none-eabi-gcc -o "CxpiLoopbackFreertos_s32k146.elf" "@CxpiLoopbackFreertos_s32k146.args"  $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

CxpiLoopbackFreertos_s32k146.siz: CxpiLoopbackFreertos_s32k146.elf
	@echo 'Invoking: Standard S32DS Print Size'
	arm-none-eabi-size --format=berkeley CxpiLoopbackFreertos_s32k146.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ./*
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
