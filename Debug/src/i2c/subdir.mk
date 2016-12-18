################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/i2c/I2Cdev.cpp 

OBJS += \
./src/i2c/I2Cdev.o 

CPP_DEPS += \
./src/i2c/I2Cdev.d 


# Each subdirectory must supply rules for building sources it contributes
src/i2c/%.o: ../src/i2c/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I"/home/andrea/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


