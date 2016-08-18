////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
// 

// #ifndef RTARDULINK_MODE

#include "RTFusionRTQF.h"

//  Axis rotation array

// #ifdef RTIMU_XNORTH_YEAST
RTFLOAT RTFusionRTQF::m_axisRotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
// #endif
// RTFLOAT RTIMU::m_axisRotation[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1}; // when'RTIMU_XSOUTH_YWEST


#ifdef USE_SLERP
//  The slerp power valule controls the influence of the measured state to correct the predicted state
//  0 = measured state ignored (just gyros), 1 = measured state overrides predicted state.
//  In between 0 and 1 mixes the two conditions
#define RTQF_SLERP_POWER (RTFLOAT)0.02;

#else //  The QVALUE affects the gyro response.
#define RTQF_QVALUE	(RTFLOAT)0.001
// #define RTQF_RVALUE	(RTFLOAT)0.0005
//  The RVALUE controls the influence of the accels and compass.
//  The bigger RVALUE, the more sluggish the response.
#endif

RTFusionRTQF::RTFusionRTQF()
{
   // merged':: gyroBiasInit(); // sets up gyro bias calculation

  //  Note: (gyro,accelo) sample-Rates interact with the lpf settings 
   // eg. MPU9250_GYRO_LPF_3600 0x10 ' MPU9250_ACCEL_LPF_1130 0x08 'MPU9150_LPF_256: 0x1a

  int rate=50;
   // MPU9150_SAMPLERATE_MAX  1K ; // MPU9250_SAMPLERATE_MAX  32K ''MIN: 5
   // defaults: m_MPU9150GyroAccelLpf = MPU9150_LPF_20;
   // defaults: m_MPU9250GyroLpf = MPU9250_GYRO_LPF_41;
   // L3GD20H_SAMPLERATE_50 for GD20HM303D'gyro' LSM303D_ACCEL_SAMPLERATE_50' LSM303D_ACCEL_LPF_50;'
   // LSM303DLHC_ACCEL_SAMPLERATE_50' L3GD20_SAMPLERATE_95'gyro'GD20M303DLHC
   // LSM9DS0_ACCEL_SAMPLERATE_50 'LSM9DS0_GYRO_SAMPLERATE_95
   // LSM9DS1_ACCEL_SAMPLERATE_119 'LSM9DS1_GYRO_SAMPLERATE_119
   // BMX055_ACCEL_SAMPLERATE_125 'BMX055_GYRO_SAMPLERATE_100_32
  
  // qv': RTIMUMPU9250.cpp' RTIMUMPU9250::setSampleRate(int rate)
  if (rate >= 1000)
     m_sampleRate = 1000;

   else if (rate < 1000) {
        int sampleDiv = (1000 / rate) - 1;
        m_sampleRate = 1000 / (1 + sampleDiv);
   }
   m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
  
    m_gyroSampleCount = 0; // number of gyro samples used wn':: handleGyroBias() 'eg' m_gyroSampleCount++;
    m_gyroContinuousAlpha = 0.01f / m_sampleRate;
    m_gyroAlphaLearningRate = 2.0f / m_sampleRate; // gyro bias rapid learning rate
#ifdef USE_SLERP
    m_slerpPower = (RTFLOAT)0.02; // RTQF_SLERP_POWER;
#else
    m_Q = (RTFLOAT)0.001; // RTQF_QVALUE;
    m_R = (RTFLOAT)0.0005; // RTQF_RVALUE;
#endif
    m_enableGyro = true;
    m_enableAccel = true;
    m_enableCompass = false; // excl'compass
    // reset();
    m_firstTime = true;
    m_fusionPose = RTVector3();
    m_fusionQPose.fromEuler(m_fusionPose);
    m_measuredPose = RTVector3();
    m_measuredQPose.fromEuler(m_measuredPose);
    /*
    m_gyro = RTVector3();
    m_accel = RTVector3();
    m_compass = RTVector3();
    m_sampleNumber = 0; */
}

RTFusionRTQF::~RTFusionRTQF()
{
}

// void RTFusionRTQF::reset(){ }

// void RTFusionRTQF::newIMUData(const RTVector3& gyro, const RTVector3& accel, const RTVector3& compass, unsigned long timestamp)
// excl'compass'
void RTFusionRTQF::newIMUData(const RTVector3& gyro, const RTVector3& accel, uint64_t timestamp)
{
    // printf: "IMU update delta time: %f, sample %d\n", m_timeDelta, m_sampleNumber++ // wn' RTIMULib.git

   // if (m_enableGyro ==true) // TRUE.
    RTVector3 mf_gyro = gyro; // see RTIMULib.git: 'renamed from RTVector3 fusionGyro;

    if (m_firstTime) {
        m_lastFusionTime = timestamp;
        // calculatePose(accel, compass); //excl'compass
	calculatePoseAccel(accel);

        //  initialize the poses

        m_fusionQPose.fromEuler(m_measuredPose);
        m_fusionPose = m_measuredPose;
        m_firstTime = false;
    } else {
        m_timeDelta = (RTFLOAT)(timestamp - m_lastFusionTime) / (RTFLOAT)1000000; //sec
        m_lastFusionTime = timestamp;
        if (m_timeDelta <= 0)
            return;

        // calculatePose(accel, compass); //excl'compass
	calculatePoseAccel(accel);

//      predict();

        RTFLOAT x2, y2, z2;
        RTFLOAT qs, qx, qy,qz;

        qs = m_fusionQPose.scalar();
        qx = m_fusionQPose.x();
        qy = m_fusionQPose.y();
        qz = m_fusionQPose.z();


        x2 = mf_gyro.x() / (RTFLOAT)2.0;
        y2 = mf_gyro.y() / (RTFLOAT)2.0;
        z2 = mf_gyro.z() / (RTFLOAT)2.0;

        // Predict new state

        m_fusionQPose.setScalar(qs + (-x2 * qx - y2 * qy - z2 * qz) * m_timeDelta);
        m_fusionQPose.setX(qx + (x2 * qs + z2 * qy - y2 * qz) * m_timeDelta);
        m_fusionQPose.setY(qy + (y2 * qs - z2 * qx + x2 * qz) * m_timeDelta);
        m_fusionQPose.setZ(qz + (z2 * qs + y2 * qx - x2 * qy) * m_timeDelta);

//      update();

#ifdef USE_SLERP
        // if (m_enableCompass || m_enableAccel) {
		if (m_enableAccel) {

            // calculate rotation delta

            m_rotationDelta = m_fusionQPose.conjugate() * m_measuredQPose;
            m_rotationDelta.normalize();

            // take it to the power (0 to 1) to give the desired amount of correction

            RTFLOAT theta = acos(m_rotationDelta.scalar());

            RTFLOAT sinPowerTheta = sin(theta * m_slerpPower);
            RTFLOAT cosPowerTheta = cos(theta * m_slerpPower);

            m_rotationUnitVector.setX(m_rotationDelta.x());
            m_rotationUnitVector.setY(m_rotationDelta.y());
            m_rotationUnitVector.setZ(m_rotationDelta.z());
            m_rotationUnitVector.normalize();

            m_rotationPower.setScalar(cosPowerTheta);
            m_rotationPower.setX(sinPowerTheta * m_rotationUnitVector.x());
            m_rotationPower.setY(sinPowerTheta * m_rotationUnitVector.y());
            m_rotationPower.setZ(sinPowerTheta * m_rotationUnitVector.z());
            m_rotationPower.normalize();

            //  multiple this by predicted value to get result

            m_fusionQPose *= m_rotationPower;
			//'+ m_fusionQPose.normalize();
        }
#else
        if (m_enableAccel) 
            m_stateQError = m_measuredQPose - m_fusionQPose;
        else {
            m_stateQError = RTQuaternion();
        }
        // make new state estimate
        RTFLOAT qt = m_Q * m_timeDelta;
		
        m_fusionQPose += m_stateQError * (qt / (qt + m_R));
#endif

        m_fusionQPose.normalize();
        m_fusionQPose.toEuler(m_fusionPose);
		// ROS_INFO': "RTQF quat", m_fusionQPose; (kalman4: "Error quat", m_stateQError)
		// RTMath::displayRadians("RTQF pose", m_fusionPose)
		// RTMath::displayRadians("Measured pose", m_measuredPose)
    }
}

// void RTFusionRTQF::calculatePose(const RTVector3& accel, const RTVector3& mag)
//excl'compass
void RTFusionRTQF::calculatePoseAccel(const RTVector3& accel)
{
    RTQuaternion m;
    RTQuaternion q;

    // bool compassValid = (mag.x() != 0) || (mag.y() != 0) || (mag.z() != 0);

    accel.accelToEuler(m_measuredPose);
    // else if (!m_enableAccel) 
    ///  m_measuredPose = m_fusionPose;

    // if (m_enableCompass && compassValid) {} // else:
	
    m_measuredPose.setZ(m_fusionPose.z());

    m_measuredQPose.fromEuler(m_measuredPose);

    //  check for quaternion aliasing. If the quaternion has the wrong sign
    //  the kalman filter will be very unhappy.

    int maxIndex = -1;
    RTFLOAT maxVal = -1000;

    for (int i = 0; i < 4; i++) {
        if (fabs(m_measuredQPose.data(i)) > maxVal) {
            maxVal = fabs(m_measuredQPose.data(i));
            maxIndex = i;
        }
    }

    //  if the biggest component has a different sign in the measured and kalman poses,
    //  change the sign of the measured pose to match.

    if (((m_measuredQPose.data(maxIndex) < 0) && (m_fusionQPose.data(maxIndex) > 0)) ||
            ((m_measuredQPose.data(maxIndex) > 0) && (m_fusionQPose.data(maxIndex) < 0))) {
        m_measuredQPose.setScalar(-m_measuredQPose.scalar());
        m_measuredQPose.setX(-m_measuredQPose.x());
        m_measuredQPose.setY(-m_measuredQPose.y());
        m_measuredQPose.setZ(-m_measuredQPose.z());
        m_measuredQPose.toEuler(m_measuredPose);
    }
}

//  assumes that this is the first thing called after axis swapping
//  for each specific IMU chip has occurred.

void RTFusionRTQF::handleGyroBias()
{
  // do axis rotation if necessary

  //RTIMULIB.git default'key:RTIMULIB_AXIS_ROTATION// m_settings->m_axisRotation = RTIMU_XNORTH_YEAST (==0);
#ifndef  RTIMU_XNORTH_YEAST // RTIMULIB-Arduino.git 
   //ie. wrt' m_settings->m_axisRotation > RTIMU_XNORTH_YEAST (==0)
    // need to do an axis rotation
    float *matrix = m_axisRotation;
    RTVector3 tempGyro = m_gyro;
    RTVector3 tempAccel = m_accel;
    RTVector3 tempCompass = m_compass;

    // do new x value
    if (matrix[0] != 0) {
        m_gyro.setX(tempGyro.x() * matrix[0]);
        m_accel.setX(tempAccel.x() * matrix[0]);
        m_compass.setX(tempCompass.x() * matrix[0]);
    } else if (matrix[1] != 0) {
        m_gyro.setX(tempGyro.y() * matrix[1]);
        m_accel.setX(tempAccel.y() * matrix[1]);
        m_compass.setX(tempCompass.y() * matrix[1]);
    } else if (matrix[2] != 0) {
        m_gyro.setX(tempGyro.z() * matrix[2]);
        m_accel.setX(tempAccel.z() * matrix[2]);
        m_compass.setX(tempCompass.z() * matrix[2]);
    }

    // do new y value
    if (matrix[3] != 0) {
        m_gyro.setY(tempGyro.x() * matrix[3]);
        m_accel.setY(tempAccel.x() * matrix[3]);
        m_compass.setY(tempCompass.x() * matrix[3]);
    } else if (matrix[4] != 0) {
        m_gyro.setY(tempGyro.y() * matrix[4]);
        m_accel.setY(tempAccel.y() * matrix[4]);
        m_compass.setY(tempCompass.y() * matrix[4]);
    } else if (matrix[5] != 0) {
        m_gyro.setY(tempGyro.z() * matrix[5]);
        m_accel.setY(tempAccel.z() * matrix[5]);
        m_compass.setY(tempCompass.z() * matrix[5]);
    }

    // do new z value
    if (matrix[6] != 0) {
        m_gyro.setZ(tempGyro.x() * matrix[6]);
        m_accel.setZ(tempAccel.x() * matrix[6]);
        m_compass.setZ(tempCompass.x() * matrix[6]);
    } else if (matrix[7] != 0) {
        m_gyro.setZ(tempGyro.y() * matrix[7]);
        m_accel.setZ(tempAccel.y() * matrix[7]);
        m_compass.setZ(tempCompass.y() * matrix[7]);
    } else if (matrix[8] != 0) {
        m_gyro.setZ(tempGyro.z() * matrix[8]);
        m_accel.setZ(tempAccel.z() * matrix[8]);
        m_compass.setZ(tempCompass.z() * matrix[8]);
    }
#endif
   // if (!m_gyroBiasValid) {
       // re'excluded'if-statement wn'RTIMULib.git'
        RTVector3 deltaAccel = m_previousAccel;
        deltaAccel -= m_accel;   // compute difference
        m_previousAccel = m_accel;

        if ((deltaAccel.squareLength() < RTIMU_FUZZY_ACCEL_ZERO_SQUARED) &&
            (m_gyro.squareLength() < RTIMU_FUZZY_GYRO_ZERO_SQUARED)) 
        {
            // what we are seeing on the gyros should be bias only 'so learn from this
            m_gyroBias.setX((1.0 - m_gyroAlphaLearningRate) * m_gyroBias.x() + m_gyroAlphaLearningRate * m_gyro.x());
            m_gyroBias.setY((1.0 - m_gyroAlphaLearningRate) * m_gyroBias.y() + m_gyroAlphaLearningRate * m_gyro.y());
            m_gyroBias.setZ((1.0 - m_gyroAlphaLearningRate) * m_gyroBias.z() + m_gyroAlphaLearningRate * m_gyro.z());

            if (m_gyroSampleCount < (5 * m_sampleRate)) {
                m_gyroSampleCount++;

                if (m_gyroSampleCount == (5 * m_sampleRate)) 
                // this could have been true 'already 'of course
                    m_gyroBiasValid = true;
            }
        }
    // } // end-of "if (!m_gyroBiasValid) {} " // re'excluded'if-statement wn'RTIMULib.git'
    
    m_gyro -= m_gyroBias;
}
    
// Accelerometer residuals (g):
// RTVector3 RTFusionRTQF::getAccelResiduals()   // m_accel ;
RTVector3 RTFusionRTQF::getAccelResiduals(RTVector3 rt_accel)
{
    RTVector3 residuals;

    RTQuaternion rotatedGravity;
    RTQuaternion fusedConjugate;
    RTQuaternion qTemp;

    //  do gravity rotation and subtraction
    m_gravity.setScalar(0);
    m_gravity.setX(0);
    m_gravity.setY(0);
    m_gravity.setZ(1);

    // create the conjugate of the QPose
    fusedConjugate = m_fusionQPose.conjugate();

    // do the rotation - takes two steps with qTemp as the intermediate variable
    qTemp = m_gravity * m_fusionQPose;
    rotatedGravity = fusedConjugate * qTemp;

    // now adjust the measured accel and change the signs to make sense

    residuals.setX(-(rt_accel.x() - rotatedGravity.x()));
    residuals.setY(-(rt_accel.y() - rotatedGravity.y()));
    residuals.setZ(-(rt_accel.z() - rotatedGravity.z()));
    return residuals;
}

	/*
	 * // ie. see: RTIMULib-Arduino.git: ArduinoAccel.ino 'Ln80s
	// adjust the measured accel and change the signs to make sense
        //  do gravity rotation and subtraction
        rt_fusedConjugate = rt_fusion.getFusionQPose().conjugate();
        // do the rotation - takes two steps
        rt_rotatedGravity = rt_fusedConjugate * ( rt_gravity * rt_fusion.getFusionQPose() );
  //with' 
        rt_residuAccel.setX( vf_ax - rotatedGravity.x() ) *(-1.0) );
        rt_residuAccel.setY( vf_ay - rotatedGravity.y() ) *(-1.0) );
        rt_residuAccel.setZ( vf_ay - rotatedGravity.y() ) *(-1.0) ); //

	robbase_msg::RazorImu rz_residuAccel;
        rz_residuAccel.roll = rt_residuAccel.x();
        rz_residuAccel.pitch = rt_residuAccel.y() ;
        rz_residuAccel.yaw = rt_residuAccel.z();
 // or'
	robbase_msg::RazorImu rz_residuAccel;
        rz_residuAccel.roll = vf_ax - rotatedGravity.x() ) *(-1.0) ;
        rz_residuAccel.pitch = vf_ay - rotatedGravity.y() ) *(-1.0) ;
        rz_residuAccel.yaw = vf_az - rotatedGravity.z() ) *(-1.0);

        rt_residuAccel.setX( rz_residuAccel.roll);
        rt_residuAccel.setY( rz_residuAccel.pitch);
        rt_residuAccel.setZ( rz_residuAccel.yaw); //

	* */

// RTVector3 residuals = RTIMU *m_imu->getAccelResiduals();

// #endif // #ifndef RTARDULINK_MODE
