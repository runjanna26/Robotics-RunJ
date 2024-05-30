################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Libs/AS5048Ainterface.cpp \
../Core/Libs/CurrentSense.cpp \
../Core/Libs/foc_utils.cpp \
../Core/Libs/lowpass_filter.cpp \
../Core/Libs/motor_param.cpp \
../Core/Libs/pid.cpp \
../Core/Libs/pwm_drivers.cpp \
../Core/Libs/simpleFOC.cpp 

OBJS += \
./Core/Libs/AS5048Ainterface.o \
./Core/Libs/CurrentSense.o \
./Core/Libs/foc_utils.o \
./Core/Libs/lowpass_filter.o \
./Core/Libs/motor_param.o \
./Core/Libs/pid.o \
./Core/Libs/pwm_drivers.o \
./Core/Libs/simpleFOC.o 

CPP_DEPS += \
./Core/Libs/AS5048Ainterface.d \
./Core/Libs/CurrentSense.d \
./Core/Libs/foc_utils.d \
./Core/Libs/lowpass_filter.d \
./Core/Libs/motor_param.d \
./Core/Libs/pid.d \
./Core/Libs/pwm_drivers.d \
./Core/Libs/simpleFOC.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Libs/%.o Core/Libs/%.su Core/Libs/%.cyclo: ../Core/Libs/%.cpp Core/Libs/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/WINDOWS 11/Documents/Robotics-RunJ/bldc_driver/control/bldc_driver_controller_runj/Core/Libs" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Libs

clean-Core-2f-Libs:
	-$(RM) ./Core/Libs/AS5048Ainterface.cyclo ./Core/Libs/AS5048Ainterface.d ./Core/Libs/AS5048Ainterface.o ./Core/Libs/AS5048Ainterface.su ./Core/Libs/CurrentSense.cyclo ./Core/Libs/CurrentSense.d ./Core/Libs/CurrentSense.o ./Core/Libs/CurrentSense.su ./Core/Libs/foc_utils.cyclo ./Core/Libs/foc_utils.d ./Core/Libs/foc_utils.o ./Core/Libs/foc_utils.su ./Core/Libs/lowpass_filter.cyclo ./Core/Libs/lowpass_filter.d ./Core/Libs/lowpass_filter.o ./Core/Libs/lowpass_filter.su ./Core/Libs/motor_param.cyclo ./Core/Libs/motor_param.d ./Core/Libs/motor_param.o ./Core/Libs/motor_param.su ./Core/Libs/pid.cyclo ./Core/Libs/pid.d ./Core/Libs/pid.o ./Core/Libs/pid.su ./Core/Libs/pwm_drivers.cyclo ./Core/Libs/pwm_drivers.d ./Core/Libs/pwm_drivers.o ./Core/Libs/pwm_drivers.su ./Core/Libs/simpleFOC.cyclo ./Core/Libs/simpleFOC.d ./Core/Libs/simpleFOC.o ./Core/Libs/simpleFOC.su

.PHONY: clean-Core-2f-Libs

