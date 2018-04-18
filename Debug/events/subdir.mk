################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../events/MyEvent.cpp 

OBJS += \
./events/MyEvent.o 

CPP_DEPS += \
./events/MyEvent.d 


# Each subdirectory must supply rules for building sources it contributes
events/%.o: ../events/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -DBOOST_LOG_DYN_LINK -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


