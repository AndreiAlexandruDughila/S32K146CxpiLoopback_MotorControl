################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/Cxpi.c \
../src/Lpuart.c \
../src/Pwm_Ftm.c \
../src/main.c \
../src/rtos.c 

OBJS += \
./src/Cxpi.o \
./src/Lpuart.o \
./src/Pwm_Ftm.o \
./src/main.o \
./src/rtos.o 

C_DEPS += \
./src/Cxpi.d \
./src/Lpuart.d \
./src/Pwm_Ftm.d \
./src/main.d \
./src/rtos.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@src/Cxpi.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


