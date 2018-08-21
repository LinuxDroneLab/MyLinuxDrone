/*
 * MyPIDCntrllr.cpp
 *
 *  Created on: 11 set 2017
 *      Author: andrea
 */

#include <agents/MyPIDCntrllr.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/log/trivial.hpp>
#include <events/MyIMUSample.h>
#include <events/MyBaroSample.h>
#include <events/MyOutMotors.h>
#include <events/MyYPRError.h>
#include <events/MyRCSample.h>
#include <events/MyMotorsDisarmed.h>
#include <events/MyMotorsArmed.h>
#include <iostream>
#include <syslog.h>

#define pi 3.1415926
#define pi2 1.5707963

RangeFloat MyPIDCntrllr::TARGET_RANGES[] = { RangeFloat(-62.0f, 62.0f), // roll
                                             RangeFloat(-62.0f, 62.0f), // pitch
                                             RangeFloat(-125.0f, 125.0f), // yaw
                                             RangeFloat(1000.0f, 2000.0f), // thrust
                                             RangeFloat(0.0f, 100.0f), // aux1
                                             RangeFloat(0.0f, 100.0f)  // aux2
        };
ValueFloat MyPIDCntrllr::TARGET_VALUES[] = {
        ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[ROLL_POS]),
        ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[PITCH_POS]),
        ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[YAW_POS]),
        ValueFloat(1000.0f, MyPIDCntrllr::TARGET_RANGES[THRUST_POS]),
        ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[AUX1_POS]),
        ValueFloat(0.0f, MyPIDCntrllr::TARGET_RANGES[AUX2_POS])
};
int8_t MyPIDCntrllr::QUATERNION_DIRECTION_RPY[] = { 1, -1, 1 };
int8_t MyPIDCntrllr::RC_DIRECTION_RPY[] = { 1, -1, 1 };
float MyPIDCntrllr::FREQUENCY = 100.0f;
RangeFloat MyPIDCntrllr::INTEGRAL_RANGE = RangeFloat(-10.0f, 10.0f);
RangeFloat MyPIDCntrllr::ALTITUDE_RANGE = RangeFloat(-20000.0f, 20000.0f);
using namespace std;

MyPIDCntrllr::MyPIDCntrllr(boost::shared_ptr<MyEventBus> bus,
                           vector<MyEvent::EventType> acceptedEventTypes) :
        MyAgent(bus, acceptedEventTypes), initialized(false), armed(false), yawErr(
                1.0f, 4, 10, 3, INTEGRAL_RANGE), pitchErr(1.0f, 4, 10, 10,
                                                          INTEGRAL_RANGE), rollErr(
                1.0f, 4, 10, 10, INTEGRAL_RANGE), altitudeBuff(1.0f, 20, 2, 2,
                                                          ALTITUDE_RANGE)
{
    this->clean();
    /*
     * Una configurazione che sembra andare:
     * keRoll/kePitch = 1.11f;
     * keDRoll/keDPitch = 9.8f;
     * keIRoll/keIPitch = 0.04080f;
     * keYaw = 2.41f; <-- con questo qualcosa non va. Senza di esso il decollo OK, ma gira su sé stesso.
     * Modificare controllo Yaw in modo che identifichi la velocità di rotazione e non i gradi target
     */

    keRoll = 1.44f; //1.11f; //1.48f //1.90f; //3.2f; //0.45f;
    keIRoll = 0.0f; //0.04080f; //0.026f; //0.000523f; //0.028f; //0.000523f;
    keDRoll = 4.10f; // 9.8f; // 5.56f ok senza yaw; //3.0f; // 3.8f; //5.5f; //15.5f; // 4.0f; //0.012f; //2.0f;

    kePitch = 1.44; // 1.11f; //1.48f //1.90f; //3.2f; //0.45f;
    keIPitch = 0.0f; //0.0480f; //0.026f; //0.000523f; //0.028f; //0.000523f;
    keDPitch = 4.10f; // 9.8f;//5.56f ok senza yaw; //3.0f; // 3.8f; //5.5f; //15.5f; //4.0f; //0.012f; //2.0f;

    keYaw = 2.41f; //1.5f; // 8.0f; //0.05f;
    keIYaw = 0.0f; //0.116f;
    keDYaw = 0.0f; //1.3f; // 6.3f;

    // TODO: usare parametro diverso per yaw. La rotazione richiede molti più giri
    // modificare di conseguenza la funzione calcOutput
    deg2MicrosFactor = 200.0f; //350.0f;
    deg2MicrosYawFactor = 200.0f; //60.0f;

    baroData.altitude = 0.0f;
    baroData.pressure = 0;
    baroData.temperature = 0.0f;
    imuSampleTimeStampMillis = 0;
}

MyPIDCntrllr::~MyPIDCntrllr()
{

}

void MyPIDCntrllr::initialize()
{
    initialized = true;
    syslog(LOG_INFO, "mydrone: MyPIDCntrllr: initialization OK");

}

MyPIDCntrllr::YPRT MyPIDCntrllr::calcYPRData(boost::math::quaternion<float> q)
{
    YPRT result = { };
    float real = q.real();
    float x = q.R_component_2();
    float y = q.R_component_3();
    float z = q.R_component_4();

    result.yaw = QUATERNION_DIRECTION_RPY[YAW_POS]
            * atan2(2.0f * (real * z + x * y), 1.0f - 2.0f * (y * y + z * z))
            * 57.295779513f;
    result.pitch = QUATERNION_DIRECTION_RPY[PITCH_POS]
            * asin(2.0f * real * y - 2.0 * z * x) * 57.295779513f;
    result.roll = QUATERNION_DIRECTION_RPY[ROLL_POS]
            * atan2(2.0f * (real * x + y * z), 1.0f - 2.0f * (x * x + y * y))
            * 57.295779513f;
//	syslog(LOG_INFO, "YPR: y(%3.2f), p(%3.2f), r(%3.2f)", result.yaw, result.pitch, result.roll);
    return result;
}

void MyPIDCntrllr::calcErr(YPRT &yprtReq, YPRT &yprtReal)
{
    // considero errore limitato a 10 deg.
    float yawErrTmp = std::min<float>(
            10.0f,
            std::max<float>(-10.0f, yprtReal.yaw - yprtReq.yaw));
    yawErr.push(yawErrTmp);
    pitchErr.push(
            std::min<float>(
                    10.0f,
                    std::max<float>(-10.0f, yprtReal.pitch - yprtReq.pitch)));
    rollErr.push(
            std::min<float>(
                    10.0f,
                    std::max<float>(-10.0f, yprtReal.roll - yprtReq.roll)));
//	syslog(LOG_INFO, "EYPRT: y(%3.5f), p(%3.5f,%3.5f), r(%3.5f,%3.5f)", yawErr.getMean(), pitchErr.getMean(), pitchErr.getIntegral(), rollErr.getMean(), rollErr.getIntegral());
//    syslog(LOG_INFO, "EYPRT: p(%3.5f, %3.5f, %3.5f, %3.5f), r(%3.5f, %3.5f, %3.5f, %3.5f), t(%5.5f)", yprtReal.pitch, pitchErr.getMean(), pitchErr.getIntegral(), pitchErr.getDerivate(), yprtReal.roll, rollErr.getMean(), rollErr.getIntegral(), rollErr.getDerivate(), yprtReal.thrust);

}

// The YPR input is the distance from real to target data with PID correction.
// the thrust is absolute value
MyPIDCntrllr::YPRT MyPIDCntrllr::calcCorrection(YPRT &yprt)
{
    YPRT result = { 0.0f, 0.0f, 0.0f, 0.0f };
    float yawCorr = yawErr.getMean() * keYaw + yawErr.getIntegral() * keIYaw
            + yawErr.getDerivate() * keDYaw;
    float pitchCorr = pitchErr.getMean() * kePitch
            + pitchErr.getIntegral() * keIPitch
            + pitchErr.getDerivate() * keDPitch;
    float rollCorr = rollErr.getMean() * keRoll
            + rollErr.getIntegral() * keIRoll + rollErr.getDerivate() * keDRoll;

// FIXME: non mi pare funzioni
//    result.yaw = yprt.yaw - yawCorr;
//    result.pitch = yprt.pitch - pitchCorr;
//    result.roll = yprt.roll - rollCorr;
//    result.thrust = yprt.thrust;

    result.yaw = -yawCorr;
    result.pitch = -pitchCorr;
    result.roll = -rollCorr;
    result.thrust = yprt.thrust;

    // syslog(LOG_INFO, "y=(%5.5f), pI=(%5.5f), rI=(%5.5f), t=(%5.5f)", yawCorr, pitchErr.getIntegral(), rollErr.getIntegral(), yprt.thrust);

    return result;
}

MyPIDCntrllr::YPRT MyPIDCntrllr::calcDelta(YPRT &yprt1, YPRT &yprt2)
{
    YPRT result = yprt1 - yprt2;
    return result;
}

// TODO: spostare su MyMotorsAgent
MyPIDCntrllr::PIDOutput MyPIDCntrllr::calcOutput(YPRT &data)
{
    /* 3      1
         \  /
          \/
          /\
	     /  \
	   2      4
     */
    // F3 e F4 girano in senso orario (CW)
    // F1 + F3 - F2 - F4 = pitch
    // F2 + F3 - F1 - F4 = roll
    // F3 + F4 - F1 - F2 = yaw
    // F1 + F2 + F3 + F4 = thrust
    // +--+--+--+--+   +--+   +--+
    // |-1|-1| 1| 1|   |F1|   |Y |
    // +--+--+--+--+   +--+   +--+
    // | 1|-1| 1|-1|   |F2|   |P |
    // +--+--+--+--+ * +--+ = +--+
    // |-1| 1| 1|-1|   |F3|   |R |
    // +--+--+--+--+   +--+   +--+
    // | 1| 1| 1| 1|   |F4|   |T |
    // +--+--+--+--+   +--+   +--+
    /*
     -->inv(M)
     +-----+-----+-----+-----+   +---+   +----+
     |-0.25| 0.25|-0.25| 0.25|   | Y |   | F1 |
     +-----+-----+-----+-----+   +---+   +----+
     |-0.25|-0.25| 0.25| 0.25|   | P |   | F2 |
     +-----+-----+-----+-----+ * +---+ = +----+
     | 0.25| 0.25| 0.25| 0.25|   | R |   | F3 |
     +-----+-----+-----+-----+   +---+   +----+
     | 0.25|-0.25|-0.25| 0.25|   | T |   | F4 |
     +-----+-----+-----+-----+   +---+   +----+
     */
    // Transform input (delta attitude and thrust) to output (nanoseconds for motors)
    long f1 = std::max<long>(
            1000000L,
            std::min<long>(
                    2000000L,
                    std::lrint(
                            (data.thrust
                                    + ((0.25) * data.pitch * deg2MicrosFactor
                                            + (-0.25) * data.roll
                                                    * deg2MicrosFactor
                                            + (-0.25) * data.yaw
                                                    * deg2MicrosYawFactor))
                                    * 1000.0f)));
    long f2 = std::max<long>(
            1000000L,
            std::min<long>(
                    2000000L,
                    std::lrint(
                            (data.thrust
                                    + ((-0.25) * data.pitch * deg2MicrosFactor
                                            + (0.25) * data.roll
                                                    * deg2MicrosFactor
                                            + (-0.25) * data.yaw
                                                    * deg2MicrosYawFactor))
                                    * 1000.0f)));
    long f3 = std::max<long>(
            1000000L,
            std::min<long>(
                    2000000L,
                    std::lrint(
                            (data.thrust
                                    + ((0.25) * data.pitch * deg2MicrosFactor
                                            + (0.25) * data.roll
                                                    * deg2MicrosFactor
                                            + (0.25) * data.yaw
                                                    * deg2MicrosYawFactor))
                                    * 1000.0f)));
    long f4 = std::max<long>(
            1000000L,
            std::min<long>(
                    2000000L,
                    std::lrint(
                            (data.thrust
                                    + ((-0.25) * data.pitch * deg2MicrosFactor
                                            + (-0.25) * data.roll
                                                    * deg2MicrosFactor
                                            + (0.25) * data.yaw
                                                    * deg2MicrosYawFactor))
                                    * 1000.0f)));

    MyPIDCntrllr::PIDOutput result = { };
    result.front = uint16_t(f1/320); // in range [3125,6250]
    result.rear = uint16_t(f2/320);
    result.left = uint16_t(f3/320);
    result.right = uint16_t(f4/320);
//	syslog(LOG_INFO, "MOT: f(%ld), r(%ld), l(%ld), r(%ld), y(%5.5f), p(%5.5f), r(%5.5f), t(%5.5f), tt(%5.5f)", f1, f2, f3, f4, data.yaw, data.pitch, data.roll, data.thrust, targetData.thrust);
    return result;
}
void MyPIDCntrllr::sendOutput(PIDOutput &data)
{
    { // out error event
        boost::shared_ptr<MyYPRError> evOut(
                boost::make_shared<MyYPRError>(
                        this->getUuid(), prevSample.yaw, prevSample.pitch,
                        prevSample.roll, requestedData.yaw, requestedData.pitch,
                        requestedData.roll, keRoll * rollErr.getMean(),
                        keIRoll * rollErr.getIntegral(),
                        keDRoll * rollErr.getDerivate(),
                        kePitch * pitchErr.getMean(),
                        keIPitch * pitchErr.getIntegral(),
                        keDPitch * pitchErr.getDerivate(),
                        keYaw * yawErr.getMean(), keIYaw * yawErr.getIntegral(),
                        keDYaw * yawErr.getDerivate()));
        m_signal(evOut);
    }
    {
//	    syslog(LOG_INFO, "mydrone. MOTORS: front(%ld), rear(%ld), left(%ld), right(%ld)", data.front, data.rear, data.left, data.right);

        // out state motors
        boost::shared_ptr<MyOutMotors> evOut(
                boost::make_shared<MyOutMotors>(this->getUuid(), data.front,
                                                data.rear, data.left,
                                                data.right));
        m_signal(evOut);
    }
}

MyPIDCntrllr::YPRT MyPIDCntrllr::getYPRTFromTargetData()
{
    YPRT result = { };
    result.yaw = float(
            std::trunc(
                    RC_DIRECTION_RPY[YAW_POS]
                            * MyPIDCntrllr::TARGET_VALUES[YAW_POS].getValue()));
    result.pitch =
            float(std::trunc(
                    RC_DIRECTION_RPY[PITCH_POS]
                            * MyPIDCntrllr::TARGET_VALUES[PITCH_POS].getValue()));
    result.roll =
            float(std::trunc(
                    RC_DIRECTION_RPY[ROLL_POS]
                            * MyPIDCntrllr::TARGET_VALUES[ROLL_POS].getValue()));
    result.thrust = float(
            std::round(MyPIDCntrllr::TARGET_VALUES[THRUST_POS].getValue()));

    if ((result.yaw >= -1.1f) && (result.yaw <= 1.1f))
    {
        result.yaw = 0.0f;
    }
    if ((result.pitch >= -1.1f) && (result.pitch <= 1.1f))
    {
        result.pitch = 0.0f;
    }
    if ((result.roll >= -1.1f) && (result.roll <= 1.1f))
    {
        result.roll = 0.0f;
    }
    return result;
}

void MyPIDCntrllr::processImuSample(boost::math::quaternion<float> sampleQ,
                                    float yaw, float pitch, float roll,
                                    VectorFloat gravity, VectorInt16 accel,
                                    VectorInt16 linearAccel, long int elapsedMillis)
{
    YPRT sample = { };
    sample.yaw = yaw;
    sample.pitch = pitch;
    sample.roll = roll;
    sample.thrust = requestedData.thrust; // not evaluated from IMU. I force to requested.

    // inizializza stato iniziale in seguito ad un disarm
    if (prevSample.isZero() && prevExpected.isZero())
    {
        prevSample = sample;
        prevExpected = {0.0f,0.0f,0.0f,0.0f};
    }

    YPRT nextExpected = (requestedData - sample);

    float maxPitchRollDeg = MYPIDCNTRLLR_MAX_DEG_PER_SEC/10.0f;
    float maxYawDeg = MYPIDCNTRLLR_MAX_DEG_PER_SEC_YAW/10.0f;
    nextExpected.limitYPR(maxPitchRollDeg, maxYawDeg); // Distanza max da percorrere in un decimo di secondo

    /**********************************************************************
     * Non sembra andar bene ...
     * riduce troppo quando l'errore è grande
     * aumenta troppo quando è piccolo
     */
    // Calcolo la velocità di rotazione che dovrò avere al prossimo ciclo
//    float b = ((maxYawDeg + abs(nextExpected.yaw))/maxYawDeg);
//    nextExpected.yaw = nextExpected.yaw / (b * abs(nextExpected.yaw) + 1);
//    b = ((maxPitchRollDeg + abs(nextExpected.pitch))/maxPitchRollDeg);
//    nextExpected.pitch = nextExpected.pitch / (b * abs(nextExpected.pitch) + 1);
//    b = ((maxPitchRollDeg + abs(nextExpected.roll))/maxPitchRollDeg);
//    nextExpected.roll = nextExpected.roll / (b * abs(nextExpected.roll) + 1);

    // Calcolo la velocità di rotazione che dovrò avere al prossimo ciclo
    nextExpected.divideYPR(17.0f);

    YPRT deltaReal = sample - prevSample; // Distanza percorsa nel ciclo precedente
    // normalize delta on 10 millis (100Hz)
    deltaReal.divideYPR(float(elapsedMillis)/10.0f);

    // Calcolo errore solo se sono in volo
    // TODO: trovare un modo migliore ...
//    if (sample.thrust > 1350.0f)
//    {
        calcErr(prevExpected, deltaReal);
//    } else {
//        // TODO: Da togliere. Mi serve per i test
//        calcErr(prevExpected, deltaReal);
//    }

    float alfa = ((requestedData.thrust - 1000.0f)/1000.0f)*(pi2);

    nextExpected.thrust = 1000.0f + (requestedData.thrust - 1000.0f) * sin(alfa);
    prevSample = sample;
    prevExpected = nextExpected;

    /* CALCOLO INPUT
     * calculate input data for transformation function
     * La correzione è data dal target richiesto (espresso in gradi) in un ciclo di frequenza e compensata con l'errore (gained)
     */
    YPRT input = calcCorrection(nextExpected);


    // calculate transformation function
    PIDOutput output = {};
    if(requestedData.thrust >= 1010.0f) {
        output = calcOutput(input);
    } else {
        output.clean();
    }

//    syslog(LOG_INFO, "Y(%5.5f),P(%5.5f),R(%5.5f), FR(%ld),FL(%ld),RR(%ld),RL(%ld), T(%5.5f)", nextExpected.yaw, nextExpected.pitch, nextExpected.roll, output.front - std::lrint(input.thrust), output.left - std::lrint(input.thrust), output.right  - std::lrint(input.thrust),output.rear  - std::lrint(input.thrust), input.thrust);
//    syslog(LOG_INFO, "P(%5.5f),PD(%5.5f),PI(%5.5f), FR(%ld),FL(%ld),RR(%ld),RL(%ld), T(%5.5f)", pitchErr.getMean() * deg2MicrosFactor, pitchErr.getDerivate() * keDPitch * deg2MicrosFactor, pitchErr.getIntegral() * keIPitch * deg2MicrosFactor, output.front - std::lrint(input.thrust), output.left - std::lrint(input.thrust), output.right  - std::lrint(input.thrust),output.rear  - std::lrint(input.thrust), input.thrust);

    // send to motors
        this->sendOutput(output);
}
void MyPIDCntrllr::clean()
{
    yawErr.clean();
    pitchErr.clean();
    rollErr.clean();
    requestedData.clean();
    prevSample.clean();
    prevExpected.clean();
    this->armed = false;
}
void MyPIDCntrllr::disarm()
{
    this->clean();
}
void MyPIDCntrllr::arm()
{
    this->clean();
    this->armed = true;
}
void MyPIDCntrllr::processEvent(boost::shared_ptr<MyEvent> event)
{
    if (!initialized)
    {
        initialize();
    }
    if (this->getState() == MyAgentState::Active)
    {
        if (event->getType() == MyEvent::EventType::MotorsDisarmed)
        {
            syslog(LOG_INFO, "Motors Disarmed");
            this->disarm();
        }
        else if (event->getType() == MyEvent::EventType::MotorsArmed)
        {
            syslog(LOG_INFO, "Motors Armed");
            this->arm();
        }
        else if (event->getType() == MyEvent::EventType::IMUSample
                && this->armed)
        {
            boost::shared_ptr<MyIMUSample> imuSample =
                    boost::static_pointer_cast<MyIMUSample>(event);
            long int elapsedMillis = 10;
            if(imuSampleTimeStampMillis != 0) {
                elapsedMillis = imuSample->getTimestampMillis() - imuSampleTimeStampMillis;
            }
            imuSampleTimeStampMillis = imuSample->getTimestampMillis();
            boost::math::quaternion<float> q = imuSample->getQuaternion();
            this->processImuSample(q, imuSample->getYaw(),
                                   imuSample->getPitch(), imuSample->getRoll(),
                                   imuSample->getGravity(),
                                   imuSample->getAccel(),
                                   imuSample->getLinearAccel(),
                                   elapsedMillis);
        }
        else if (event->getType() == MyEvent::EventType::RCSample)
        {
            boost::shared_ptr<MyRCSample> rcSample = boost::static_pointer_cast<
                    MyRCSample>(event);
            MyPIDCntrllr::TARGET_VALUES[ROLL_POS].setPercentValue(
                    (*rcSample).getRollPercent());
            MyPIDCntrllr::TARGET_VALUES[PITCH_POS].setPercentValue(
                    (*rcSample).getPitchPercent());
            MyPIDCntrllr::TARGET_VALUES[YAW_POS].setPercentValue(
                    (*rcSample).getYawPercent());
            MyPIDCntrllr::TARGET_VALUES[THRUST_POS].setPercentValue(
                    (*rcSample).getThrustPercent());
            MyPIDCntrllr::TARGET_VALUES[AUX1_POS].setPercentValue(
                    (*rcSample).getAux1Percent());
            MyPIDCntrllr::TARGET_VALUES[AUX2_POS].setPercentValue(
                    (*rcSample).getAux2Percent());
            requestedData = this->getYPRTFromTargetData();
//            float v1 = MyPIDCntrllr::TARGET_VALUES[AUX2_POS].getValue();
//            float vv1 = MyPIDCntrllr::TARGET_VALUES[AUX1_POS].getValue()/10.0f;

//            if(abs(vv1 - keYaw) > 0.1f) {
//                keYaw = vv1;
//                syslog(LOG_INFO, "K: E(%5.5f)",keYaw);
//            }

//            float vv1 = MyPIDCntrllr::TARGET_VALUES[AUX1_POS].getValue()/1000.0f;
//            if(abs(vv1 - keIRoll) > 0.001f) {
//                keIRoll = vv1;
//                keIPitch = vv1;
//                syslog(LOG_INFO, "K: E(%5.5f)",keIRoll);
//            }

//            if(abs(v1 - keDRoll) > 0.1f || abs(vv1 - keRoll) > 0.1f) {
//                keDRoll = v1;
//                keDPitch = keDRoll;
//                keRoll = vv1;
//                kePitch = keRoll;
//                syslog(LOG_INFO, "K: E(%5.5f),D(%5.5f)",keRoll, keDRoll);
//            }
//            syslog(LOG_INFO, "RC: Y(%5.5f),P(%5.5f),R(%5.5f),T(%5.5f),A1(%5.5f),A2(%5.5f)", requestedData.yaw, requestedData.pitch, requestedData.roll, requestedData.thrust, MyPIDCntrllr::TARGET_VALUES[AUX1_POS].getValue(), MyPIDCntrllr::TARGET_VALUES[AUX2_POS].getValue());

//            syslog(LOG_INFO, "RCSample: ts=%ld, r=%3.2f, p=%3.2f, y=%3.2f, t=%3.2f", rcSample->getTimestampMillis(), MyPIDCntrllr::TARGET_VALUES[ROLL_POS].getValue(), MyPIDCntrllr::TARGET_VALUES[PITCH_POS].getValue(), MyPIDCntrllr::TARGET_VALUES[YAW_POS].getValue(), MyPIDCntrllr::TARGET_VALUES[THRUST_POS].getValue());

        }
        else if (event->getType() == MyEvent::EventType::BaroSample)
        {
            // TODO: rivedere completamente
//            boost::shared_ptr<MyBaroSample> baroSample =
//                    boost::static_pointer_cast<MyBaroSample>(event);
//            // TODO: da spostare in metodo ad hoc per BaroPID ...
//            float prevAlt = this->altitudeBuff.getMean();
//            this->altitudeBuff.push(baroSample->getAltitude());
//            baroData.altitude = this->altitudeBuff.getMean();
//            baroData.pressure = baroSample->getPressure();
//            baroData.temperature = baroSample->getTemperature();
//            baroData.speedMetersPerSecond = (baroData.altitude - prevAlt)*1000.0f/baroSample->getDTimeMillis();
        }
        else if (event->getType() == MyEvent::EventType::IMUSample)
        {
//            boost::shared_ptr<MyIMUSample> imuSample =
//                    boost::static_pointer_cast<MyIMUSample>(event);
//            syslog(LOG_INFO, "Y(%5.5f),P(%5.5f),R(%5.5f)", imuSample->getYaw(), imuSample->getPitch(), imuSample->getRoll());

        }
        else
        {
            // skip events
//            syslog(LOG_INFO, "Skipped event");
        }
    }
}

