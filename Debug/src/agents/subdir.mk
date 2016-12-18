################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/agents/MyAgent.cpp \
../src/agents/MyClock.cpp 

OBJS += \
./src/agents/MyAgent.o \
./src/agents/MyClock.o 

CPP_DEPS += \
./src/agents/MyAgent.d \
./src/agents/MyClock.d 


# Each subdirectory must supply rules for building sources it contributes
src/agents/%.o: ../src/agents/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I"/home/andrea/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


