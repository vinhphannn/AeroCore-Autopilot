################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FC_Core/Drivers/radio/nrf24l01.c 

C_DEPS += \
./FC_Core/Drivers/radio/nrf24l01.d 

OBJS += \
./FC_Core/Drivers/radio/nrf24l01.o 


# Each subdirectory must supply rules for building sources it contributes
FC_Core/Drivers/radio/%.o FC_Core/Drivers/radio/%.su FC_Core/Drivers/radio/%.cyclo: ../FC_Core/Drivers/radio/%.c FC_Core/Drivers/radio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FC_Core-2f-Drivers-2f-radio

clean-FC_Core-2f-Drivers-2f-radio:
	-$(RM) ./FC_Core/Drivers/radio/nrf24l01.cyclo ./FC_Core/Drivers/radio/nrf24l01.d ./FC_Core/Drivers/radio/nrf24l01.o ./FC_Core/Drivers/radio/nrf24l01.su

.PHONY: clean-FC_Core-2f-Drivers-2f-radio

