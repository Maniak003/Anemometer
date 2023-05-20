################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FFTImplementationCallback.c \
../Core/Src/main.c \
../Core/Src/maxEnvHilbert.c \
../Core/Src/maxEnvHilbert_data.c \
../Core/Src/maxEnvHilbert_terminate.c \
../Core/Src/rt_nonfinite.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c 

OBJS += \
./Core/Src/FFTImplementationCallback.o \
./Core/Src/main.o \
./Core/Src/maxEnvHilbert.o \
./Core/Src/maxEnvHilbert_data.o \
./Core/Src/maxEnvHilbert_terminate.o \
./Core/Src/rt_nonfinite.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o 

C_DEPS += \
./Core/Src/FFTImplementationCallback.d \
./Core/Src/main.d \
./Core/Src/maxEnvHilbert.d \
./Core/Src/maxEnvHilbert_data.d \
./Core/Src/maxEnvHilbert_terminate.d \
./Core/Src/rt_nonfinite.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FFTImplementationCallback.cyclo ./Core/Src/FFTImplementationCallback.d ./Core/Src/FFTImplementationCallback.o ./Core/Src/FFTImplementationCallback.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/maxEnvHilbert.cyclo ./Core/Src/maxEnvHilbert.d ./Core/Src/maxEnvHilbert.o ./Core/Src/maxEnvHilbert.su ./Core/Src/maxEnvHilbert_data.cyclo ./Core/Src/maxEnvHilbert_data.d ./Core/Src/maxEnvHilbert_data.o ./Core/Src/maxEnvHilbert_data.su ./Core/Src/maxEnvHilbert_terminate.cyclo ./Core/Src/maxEnvHilbert_terminate.d ./Core/Src/maxEnvHilbert_terminate.o ./Core/Src/maxEnvHilbert_terminate.su ./Core/Src/rt_nonfinite.cyclo ./Core/Src/rt_nonfinite.d ./Core/Src/rt_nonfinite.o ./Core/Src/rt_nonfinite.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su

.PHONY: clean-Core-2f-Src

