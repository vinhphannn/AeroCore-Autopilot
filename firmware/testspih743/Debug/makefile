################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/Common/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include FC_Core/Sensors/subdir.mk
-include FC_Core/Parameters/subdir.mk
-include FC_Core/MessageBus/topics/subdir.mk
-include FC_Core/Main/subdir.mk
-include FC_Core/Estimator/ahrs/subdir.mk
-include FC_Core/Drivers/rc/subdir.mk
-include FC_Core/Drivers/rangefinder/subdir.mk
-include FC_Core/Drivers/radio/subdir.mk
-include FC_Core/Drivers/optical_flow/subdir.mk
-include FC_Core/Drivers/mag/subdir.mk
-include FC_Core/Drivers/imu/subdir.mk
-include FC_Core/Drivers/gps/subdir.mk
-include FC_Core/Drivers/barometer/subdir.mk
-include FC_Core/Drivers/actuator/subdir.mk
-include FC_Core/Drivers/subdir.mk
-include FC_Core/Controllers/mc_rate_control/subdir.mk
-include FC_Core/Controllers/mc_att_control/subdir.mk
-include FC_Core/Controllers/control_allocator/subdir.mk
-include FC_Core/Communication/subdir.mk
-include FC_Core/Commander/flight_mode_manager/subdir.mk
-include FC_Core/Commander/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CCM_DEPS)),)
-include $(CCM_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CXXM_DEPS)),)
-include $(CXXM_DEPS)
endif
ifneq ($(strip $(C++M_DEPS)),)
-include $(C++M_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := MyFC_H743
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MyFC_H743.elf \

MAP_FILES += \
MyFC_H743.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MyFC_H743.list \


# All Target
all: main-build

# Main-build Target
main-build: MyFC_H743.elf secondary-outputs

# Tool invocations
MyFC_H743.elf MyFC_H743.map: $(OBJS) $(USER_OBJS) ../STM32H743VITX_FLASH.ld makefile $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-g++ -o "MyFC_H743.elf" $(OBJS) $(USER_OBJS) $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"../STM32H743VITX_FLASH.ld" -Wl,-Map="MyFC_H743.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MyFC_H743.list: $(EXECUTABLES) makefile $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MyFC_H743.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) MyFC_H743.elf MyFC_H743.list MyFC_H743.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
