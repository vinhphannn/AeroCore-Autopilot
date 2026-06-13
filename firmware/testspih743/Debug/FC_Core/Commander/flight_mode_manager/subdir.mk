################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../FC_Core/Commander/flight_mode_manager/FlightModeManager.cpp 

OBJS += \
./FC_Core/Commander/flight_mode_manager/FlightModeManager.o 

CPP_DEPS += \
./FC_Core/Commander/flight_mode_manager/FlightModeManager.d 


# Each subdirectory must supply rules for building sources it contributes
FC_Core/Commander/flight_mode_manager/%.o FC_Core/Commander/flight_mode_manager/%.su FC_Core/Commander/flight_mode_manager/%.cyclo: ../FC_Core/Commander/flight_mode_manager/%.cpp FC_Core/Commander/flight_mode_manager/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FC_Core-2f-Commander-2f-flight_mode_manager

clean-FC_Core-2f-Commander-2f-flight_mode_manager:
	-$(RM) ./FC_Core/Commander/flight_mode_manager/FlightModeManager.cyclo ./FC_Core/Commander/flight_mode_manager/FlightModeManager.d ./FC_Core/Commander/flight_mode_manager/FlightModeManager.o ./FC_Core/Commander/flight_mode_manager/FlightModeManager.su

.PHONY: clean-FC_Core-2f-Commander-2f-flight_mode_manager

