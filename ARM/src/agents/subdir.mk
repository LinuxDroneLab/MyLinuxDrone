################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/agents/MyAgent.cpp \
../src/agents/MyClock.cpp \
../src/agents/MyDBusEmitterAgent.cpp \
../src/agents/MyIMUAgent.cpp \
../src/agents/MyMotorsAgent.cpp \
../src/agents/MyPIDCntrllr.cpp \
../src/agents/MyPIDControllerAgent.cpp \
../src/agents/MyRCAgent.cpp 

OBJS += \
./src/agents/MyAgent.o \
./src/agents/MyClock.o \
./src/agents/MyDBusEmitterAgent.o \
./src/agents/MyIMUAgent.o \
./src/agents/MyMotorsAgent.o \
./src/agents/MyPIDCntrllr.o \
./src/agents/MyPIDControllerAgent.o \
./src/agents/MyRCAgent.o 

CPP_DEPS += \
./src/agents/MyAgent.d \
./src/agents/MyClock.d \
./src/agents/MyDBusEmitterAgent.d \
./src/agents/MyIMUAgent.d \
./src/agents/MyMotorsAgent.d \
./src/agents/MyPIDCntrllr.d \
./src/agents/MyPIDControllerAgent.d \
./src/agents/MyRCAgent.d 


# Each subdirectory must supply rules for building sources it contributes
src/agents/%.o: ../src/agents/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I/hd/BBBchroot/usr/include/ -I/hd/eworkspace/pru-rc-lib/include -I/hd/BBBchroot/usr/include/glib-2.0/ -I/hd/BBBchroot/usr/include/gio-unix-2.0 -I/hd/BBBchroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I/hd/BBBchroot/usr/include/arm-linux-gnueabihf -I"/hd/eworkspace/MyDrone/src" -include/BBBchroot/usr/local/include/prussdrv.h -include/BBBchroot/usr/local/include/pruss_intc_mapping.h -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


