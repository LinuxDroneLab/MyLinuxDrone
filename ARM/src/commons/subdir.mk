################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/commons/MyUuid.cpp \
../src/commons/RangeFloat.cpp \
../src/commons/ValueFloat.cpp \
../src/commons/RangeInt16.cpp \
../src/commons/ValueInt16.cpp 

OBJS += \
./src/commons/MyUuid.o \
./src/commons/RangeFloat.o \
./src/commons/ValueFloat.o \
./src/commons/RangeInt16.o \
./src/commons/ValueInt16.o 

CPP_DEPS += \
./src/commons/MyUuid.d \
./src/commons/RangeFloat.d \
./src/commons/ValueFloat.d \
./src/commons/RangeInt16.d \
./src/commons/ValueInt16.d 


# Each subdirectory must supply rules for building sources it contributes
src/commons/%.o: ../src/commons/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I/BBBchroot/usr/local/include/ -I/BBBchroot/usr/include/glib-2.0/ -I/BBBchroot/usr/include/gio-unix-2.0 -I/BBBchroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I"/hd/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


