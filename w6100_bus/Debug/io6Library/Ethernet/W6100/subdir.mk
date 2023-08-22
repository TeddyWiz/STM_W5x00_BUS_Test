################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../io6Library/Ethernet/W6100/w6100.c 

OBJS += \
./io6Library/Ethernet/W6100/w6100.o 

C_DEPS += \
./io6Library/Ethernet/W6100/w6100.d 


# Each subdirectory must supply rules for building sources it contributes
io6Library/Ethernet/W6100/%.o io6Library/Ethernet/W6100/%.su io6Library/Ethernet/W6100/%.cyclo: ../io6Library/Ethernet/W6100/%.c io6Library/Ethernet/W6100/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../io6Library/Ethernet -I../io6Library/Ethernet/W6100 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-io6Library-2f-Ethernet-2f-W6100

clean-io6Library-2f-Ethernet-2f-W6100:
	-$(RM) ./io6Library/Ethernet/W6100/w6100.cyclo ./io6Library/Ethernet/W6100/w6100.d ./io6Library/Ethernet/W6100/w6100.o ./io6Library/Ethernet/W6100/w6100.su

.PHONY: clean-io6Library-2f-Ethernet-2f-W6100

