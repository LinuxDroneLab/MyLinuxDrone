################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/dbus/MyPIDControllerService-generated.cpp \
../src/dbus/MyPIDControllerServiceWrap.cpp 

OBJS += \
./src/dbus/MyPIDControllerService-generated.o \
./src/dbus/MyPIDControllerServiceWrap.o 

CPP_DEPS += \
./src/dbus/MyPIDControllerService-generated.d \
./src/dbus/MyPIDControllerServiceWrap.d 


# Each subdirectory must supply rules for building sources it contributes
src/dbus/%.o: ../src/dbus/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I/BBBchroot/usr/local/include/ -I/BBBchroot/usr/include/glib-2.0/ -I/BBBchroot/usr/include/gio-unix-2.0 -I/BBBchroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I"/hd/eworkspace/MyDrone/src" -include/BBBchroot/usr/local/include/prussdrv.h -include/BBBchroot/usr/local/include/pruss_intc_mapping.h -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


