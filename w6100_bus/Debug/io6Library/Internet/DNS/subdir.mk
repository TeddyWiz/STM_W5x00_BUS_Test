################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../io6Library/Internet/DNS/dns.c 

OBJS += \
./io6Library/Internet/DNS/dns.o 

C_DEPS += \
./io6Library/Internet/DNS/dns.d 


# Each subdirectory must supply rules for building sources it contributes
io6Library/Internet/DNS/%.o io6Library/Internet/DNS/%.su io6Library/Internet/DNS/%.cyclo: ../io6Library/Internet/DNS/%.c io6Library/Internet/DNS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../io6Library/Ethernet -I../io6Library/Ethernet/W6100 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-io6Library-2f-Internet-2f-DNS

clean-io6Library-2f-Internet-2f-DNS:
	-$(RM) ./io6Library/Internet/DNS/dns.cyclo ./io6Library/Internet/DNS/dns.d ./io6Library/Internet/DNS/dns.o ./io6Library/Internet/DNS/dns.su

.PHONY: clean-io6Library-2f-Internet-2f-DNS

