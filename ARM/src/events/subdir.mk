################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/events/MyAgentChangeState.cpp \
../src/events/MyAlarm.cpp \
../src/events/MyArmMotorsCmd.cpp \
../src/events/MyBaroSample.cpp \
../src/events/MyCmd.cpp \
../src/events/MyDisarmMotorsCmd.cpp \
../src/events/MyEndAccellCalCmd.cpp \
../src/events/MyEndBaroCalCmd.cpp \
../src/events/MyEndCompassCalCmd.cpp \
../src/events/MyEndGyroCalCmd.cpp \
../src/events/MyEvent.cpp \
../src/events/MyIMUSample.cpp \
../src/events/MinThrustMinPitch.cpp \
../src/events/MinThrustMaxPitch.cpp \
../src/events/MyMotorsArmed.cpp \
../src/events/MyMotorsDisarmed.cpp \
../src/events/MyOutMotors.cpp \
../src/events/MyPriorityComparator.cpp \
../src/events/MyRCSample.cpp \
../src/events/MyShutdownCmd.cpp \
../src/events/MyShutdownImmCmd.cpp \
../src/events/MyStartAccellCalCmd.cpp \
../src/events/MyStartBaroCalCmd.cpp \
../src/events/MyStartCmd.cpp \
../src/events/MyStartCompassCalCmd.cpp \
../src/events/MyStartGyroCalCmd.cpp \
../src/events/MyStopCmd.cpp \
../src/events/MyStopImmCmd.cpp \
../src/events/MyTargetSample.cpp \
../src/events/MyTick.cpp \
../src/events/MyYPRError.cpp 

OBJS += \
./src/events/MyAgentChangeState.o \
./src/events/MyAlarm.o \
./src/events/MyArmMotorsCmd.o \
./src/events/MyBaroSample.o \
./src/events/MyCmd.o \
./src/events/MyDisarmMotorsCmd.o \
./src/events/MyEndAccellCalCmd.o \
./src/events/MyEndBaroCalCmd.o \
./src/events/MyEndCompassCalCmd.o \
./src/events/MyEndGyroCalCmd.o \
./src/events/MyEvent.o \
./src/events/MyIMUSample.o \
./src/events/MinThrustMinPitch.o \
./src/events/MinThrustMaxPitch.o \
./src/events/MyMotorsArmed.o \
./src/events/MyMotorsDisarmed.o \
./src/events/MyOutMotors.o \
./src/events/MyPriorityComparator.o \
./src/events/MyRCSample.o \
./src/events/MyShutdownCmd.o \
./src/events/MyShutdownImmCmd.o \
./src/events/MyStartAccellCalCmd.o \
./src/events/MyStartBaroCalCmd.o \
./src/events/MyStartCmd.o \
./src/events/MyStartCompassCalCmd.o \
./src/events/MyStartGyroCalCmd.o \
./src/events/MyStopCmd.o \
./src/events/MyStopImmCmd.o \
./src/events/MyTargetSample.o \
./src/events/MyTick.o \
./src/events/MyYPRError.o 

CPP_DEPS += \
./src/events/MyAgentChangeState.d \
./src/events/MyAlarm.d \
./src/events/MyArmMotorsCmd.d \
./src/events/MyBaroSample.d \
./src/events/MyCmd.d \
./src/events/MyDisarmMotorsCmd.d \
./src/events/MyEndAccellCalCmd.d \
./src/events/MyEndBaroCalCmd.d \
./src/events/MyEndCompassCalCmd.d \
./src/events/MyEndGyroCalCmd.d \
./src/events/MyEvent.d \
./src/events/MyIMUSample.d \
./src/events/MinThrustMinPitch.d \
./src/events/MinThrustMaxPitch.d \
./src/events/MyMotorsArmed.d \
./src/events/MyMotorsDisarmed.d \
./src/events/MyOutMotors.d \
./src/events/MyPriorityComparator.d \
./src/events/MyRCSample.d \
./src/events/MyShutdownCmd.d \
./src/events/MyShutdownImmCmd.d \
./src/events/MyStartAccellCalCmd.d \
./src/events/MyStartBaroCalCmd.d \
./src/events/MyStartCmd.d \
./src/events/MyStartCompassCalCmd.d \
./src/events/MyStartGyroCalCmd.d \
./src/events/MyStopCmd.d \
./src/events/MyStopImmCmd.d \
./src/events/MyTargetSample.d \
./src/events/MyTick.d \
./src/events/MyYPRError.d 


# Each subdirectory must supply rules for building sources it contributes
src/events/%.o: ../src/events/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -std=c++0x -D__cplusplus=201103L -DBOOST_LOG_DYN_LINK -I/BBBchroot/usr/local/include/ -I/BBBchroot/usr/include/glib-2.0/ -I/BBBchroot/usr/include/gio-unix-2.0 -I/BBBchroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I"/hd/eworkspace/MyDrone/src" -O0 -g3 -Wall -c -fmessage-length=0 -fpermissive -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


