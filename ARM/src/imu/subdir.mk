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
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I/BBBchroot/usr/local/include/ -I/BBBchroot/usr/include/glib-2.0/ -I/BBBchroot/usr/include/gio-unix-2.0 -I/BBBchroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I"/hd/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


