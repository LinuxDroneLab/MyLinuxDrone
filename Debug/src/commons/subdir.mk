################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/commons/MyUuid.cpp 

OBJS += \
./src/commons/MyUuid.o 

CPP_DEPS += \
./src/commons/MyUuid.d 


# Each subdirectory must supply rules for building sources it contributes
src/commons/%.o: ../src/commons/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I"/home/andrea/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


