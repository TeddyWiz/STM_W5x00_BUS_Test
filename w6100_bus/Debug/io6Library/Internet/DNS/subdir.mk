################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../io6Library/Internet/DNS/dns.c 

OBJS += \
./io6Library/Internet/DNS/dns.o 

C_DEPS += \
./io6Library/Internet/DNS/dns.d 


# Each subdirectory must supply rules for building sources it contributes
io6Library/Internet/DNS/dns.o: ../io6Library/Internet/DNS/dns.c io6Library/Internet/DNS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../io6Library/Ethernet -I../io6Library/Ethernet/W6100 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"io6Library/Internet/DNS/dns.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

