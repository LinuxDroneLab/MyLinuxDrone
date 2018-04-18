################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/baro/BMP085.cpp 

OBJS += \
./src/baro/BMP085.o 

CPP_DEPS += \
./src/baro/BMP085.d 


# Each subdirectory must supply rules for building sources it contributes
src/baro/%.o: ../src/baro/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I"/home/andrea/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


