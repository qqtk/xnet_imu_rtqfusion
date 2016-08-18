////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//

#ifndef _RTFUSIONRTQF_H
#define	_RTFUSIONRTQF_H

#include "RTMath.h"
// #include "IMUDrivers/RTIMUDefs.h" : excl' imu-products' I2C Slave Addresses
// typedef struct RTIMU_DATA
// #include "RTIMULibDefs.h" // ==cf'

//  excl' class: public RTFusion as parent class
// #include "RTFusion.h"

//  Define USE_SLERP symbol to use 'more scientific prediction correction
#define USE_SLERP

//qv'"RTIMU.cpp"
//  defines the accelerometer noise level
#define RTIMU_FUZZY_GYRO_ZERO      0.20

//  defines the accelerometer noise level
#define RTIMU_FUZZY_ACCEL_ZERO      0.05
//qv'"RTIMULibDefs.h"
// fusion filter options

#define RTFUSION_TYPE_NULL                  0                   // just a dummy to keep things happy if not needed
#define RTFUSION_TYPE_KALMANSTATE4          1                   // kalman state is the quaternion pose
#define RTFUSION_TYPE_RTQF                  2                   // RT quaternion fusion

#define RTFUSION_TYPE_COUNT                 3                   // number of fusion algorithm types


typedef struct
{
    uint64_t timestamp;
    bool fusionPoseValid;
    RTVector3 fusionPose;
    bool fusionQPoseValid;
    RTQuaternion fusionQPose;
    bool gyroValid;
    RTVector3 gyro;
    bool accelValid;
    RTVector3 accel;
    bool compassValid;
    RTVector3 compass;
    bool pressureValid;
    RTFLOAT pressure;
    bool temperatureValid;
    RTFLOAT temperature;
    bool humidityValid;
    RTFLOAT humidity;
} RTIMU_DATA;

class RTFusionRTQF
{
public:
    RTFusionRTQF();
    ~RTFusionRTQF();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_RTQF; }

    // excl' reset() resets the state but keeps any setting changes (such as enables)

    // void reset();

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    void newIMUData(const RTVector3& gyro, const RTVector3& accel, const RTVector3& compass, unsigned long timestamp);
    //excl'compass' void newIMUData(const RTVector3& gyro, const RTVector3& accel, const RTVector3& compass, unsigned long timestamp);

    // control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_enableGyro = enable;}
    //void setCompassEnable(bool enable) { m_enableCompass = enable;} // excl'compass
    void setAccelEnable(bool enable) { m_enableAccel = enable; }

    // static const char *fusionName(int fusionType) { return m_fusionNameMap[fusionType]; } // "RTQF", "Kalman STATE4", "NULL".
#ifdef USE_SLERP // set the SLERP power
    void setSlerpPower(RTFLOAT power) { m_slerpPower = power; }
#else
    // can customize the noise covariance
    void setQ(RTFLOAT Q) {  m_Q = Q; reset();}
    void setR(RTFLOAT R) { if (R > 0) m_R = R; reset();}
#endif
    inline const RTVector3& getMeasuredPose() {return m_measuredPose;}
    inline const RTQuaternion& getMeasuredQPose() {return m_measuredQPose;}
    inline const RTVector3& getFusionPose() {return m_fusionPose;}
    inline const RTQuaternion& getFusionQPose() {return m_fusionQPose;}

    RTVector3 getAccelResiduals();

protected: // from' RTIMULib-Arduino.git: RTIMU.h
    // void gyroBiasInit();                                    // sets up gyro bias calculation
    void handleGyroBias();                                  // adjust gyro for bias
    static RTFLOAT m_axisRotation[9];                         // axis rotation matrix

    RTVector3 m_gyro;                                       // the gyro readings
    RTVector3 m_accel;                                      // the accel readings

    RTVector3 m_gyroBias;                                   // the recorded gyro bias 
    bool m_gyroBiasValid;                                   // true if the recorded gyro bias is valid

    RTFLOAT m_gyroAlphaLearningRate;                        // gyro bias rapid learning rate
    RTFLOAT m_gyroAlphaContinuousRate;                      // gyro bias continuous (slow) learning rate //wn [0.0, 1.0)
    int m_gyroSampleCount;                                  // number of gyro samples used
    int m_sampleRate;                                       // samples per second

    uint64_t m_sampleInterval;                              // usec vs'ms' interval between samples 
    RTVector3 m_previousAccel;                              // previous step accel for gyro learning

    
private:
   void calculatePoseAccel(const RTVector3& accel); // generates pose from accels only.
   //excl'compass 
   // void calculatePose(const RTVector3& accel, const RTVector3& mag); // generates pose from accels and heading

    RTFLOAT m_timeDelta;                                    // time between predictions
   //KF4: RTQuaternion m_stateQError;                             // difference between stateQ and measuredQ
    //excl' RTQuaternion m_stateQ;									// quaternion state vector


#ifdef USE_SLERP
    RTFLOAT m_slerpPower;                                   // a value 0 to 1 controls measured state influence
    RTQuaternion m_rotationDelta;                           // the amount : Measured state differs from predicted
    RTQuaternion m_rotationPower;                           // delta raised to the appopriate power
    RTVector3 m_rotationUnitVector;                         // the vector part of the rotation delta
#else
    RTFLOAT m_Q;                                            // process noise covariance
    RTFLOAT m_R;                                            // the measurement noise covariance
#endif

    RTQuaternion m_measuredQPose;       					// quaternion form of pose from measurement
    RTVector3 m_measuredPose;								// vector form of pose from measurement

    RTQuaternion m_fusionQPose;                             // quaternion form of pose from fusion
    RTVector3 m_fusionPose;                                 // vector form of pose from fusion

    RTQuaternion m_gravity;                                 // the gravity vector as a quaternion
    bool m_enableGyro;                                      // enables gyro as input
    bool m_enableAccel;                                     // enables accel as input
    // bool m_enableCompass;                                   // enables compass a input // ==false.
    // bool m_compassValid;                                    // true if compass data valid // not use compass
    // bool m_debug;

    bool m_firstTime;                                       // if first time after reset
    uint64_t m_lastFusionTime;                              // usec' vs'ms' unsigned long // for delta time calculation

    // static const char *m_fusionNameMap[];                   // the fusion name array // "RTQF", "Kalman STATE4", "NULL".

private:
    // void predict();
    // void update();

    // int m_sampleNumber;
};

#endif // _RTFUSIONRTQF_H
