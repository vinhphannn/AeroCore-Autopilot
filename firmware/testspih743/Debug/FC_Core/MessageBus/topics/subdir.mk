################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../FC_Core/MessageBus/topics/topics.cpp 

OBJS += \
./FC_Core/MessageBus/topics/topics.o 

CPP_DEPS += \
./FC_Core/MessageBus/topics/topics.d 


# Each subdirectory must supply rules for building sources it contributes
FC_Core/MessageBus/topics/%.o FC_Core/MessageBus/topics/%.su FC_Core/MessageBus/topics/%.cyclo: ../FC_Core/MessageBus/topics/%.cpp FC_Core/MessageBus/topics/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FC_Core-2f-MessageBus-2f-topics

clean-FC_Core-2f-MessageBus-2f-topics:
	-$(RM) ./FC_Core/MessageBus/topics/topics.cyclo ./FC_Core/MessageBus/topics/topics.d ./FC_Core/MessageBus/topics/topics.o ./FC_Core/MessageBus/topics/topics.su

.PHONY: clean-FC_Core-2f-MessageBus-2f-topics

