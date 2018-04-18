################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/imu/GY88.cpp \
../src/imu/MPU6050.cpp 

OBJS += \
./src/imu/GY88.o \
./src/imu/MPU6050.o 

CPP_DEPS += \
./src/imu/GY88.d \
./src/imu/MPU6050.d 


# Each subdirectory must supply rules for building sources it contributes
src/imu/%.o: ../src/imu/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I"/home/andrea/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


