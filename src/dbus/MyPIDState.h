/*
 * MyPIDState.h
 *
 *  Created on: 08 feb 2016
 *      Author: andrea
 */

#ifndef DBUS_MYPIDSTATE_H_
#define DBUS_MYPIDSTATE_H_
#include  <cstdint>

struct MyPID {
    uint8_t pid_enum;
    uint8_t ypr_enum;
    double value;
} ;

struct MyIMUSample {
    double current_value;
    double target_value;
    double error_;
    double error_i;
    double error_d;
};

struct MyPIDState
{
    uint32_t timestampMillis;
    MyIMUSample yawSample;
    MyIMUSample pitchSample;
    MyIMUSample rollSample;
};

#endif /* DBUS_MYPIDSTATE_H_ */
