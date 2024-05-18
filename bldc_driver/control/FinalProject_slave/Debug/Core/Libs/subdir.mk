################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Libs/HelloRunJ.cpp \
../Core/Libs/simpleFOC.cpp 

OBJS += \
./Core/Libs/HelloRunJ.o \
./Core/Libs/simpleFOC.o 

CPP_DEPS += \
./Core/Libs/HelloRunJ.d \
./Core/Libs/simpleFOC.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libs/%.o Core/Libs/%.su Core/Libs/%.cyclo: ../Core/Libs/%.cpp Core/Libs/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"/Users/runj/Documents/GitHub/Robotics-RunJ/bldc_driver/control/FinalProject_slave/Core/Libs" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Libs

clean-Core-2f-Libs:
	-$(RM) ./Core/Libs/HelloRunJ.cyclo ./Core/Libs/HelloRunJ.d ./Core/Libs/HelloRunJ.o ./Core/Libs/HelloRunJ.su ./Core/Libs/simpleFOC.cyclo ./Core/Libs/simpleFOC.d ./Core/Libs/simpleFOC.o ./Core/Libs/simpleFOC.su

.PHONY: clean-Core-2f-Libs

