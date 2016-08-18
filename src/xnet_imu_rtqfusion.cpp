// get ticks 'msg', publish /imu topic '
// and publish 'the imu/ base_link TransformStamped'msg to tf.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

#include <string>
// #include "robbase_msg/encoders.h"
#include "robbase_msg/RazorImu.h"
#include "xnet_imu_rtqfusion/xdriver.h"
// #include "xnet_imu_rtqfusion/xdriver.h"
// #define M_PI 3.1415926 // wn. colibri-imx6/usr/include/math.h: Ln.386

#include "RTMath.h"
#include "RTFusionRTQF.h" 
RTFusionRTQF rt_fusion;
RTVector3 rt_accel, rt_gyro;
RTVector3 rt_residuAccel; // Accelerometer residuals (g):

ros::Time current_time, last_time;
typedef struct{
	int gx, gy, gz;
	int ax, ay, az;
} imuRaw_vec_struct;
imuRaw_vec_struct imuRaw_struct;

float vf_gx, vf_gy, vf_gz;
float vf_ax, vf_ay, vf_az;
float vf_roll, vf_pitch, vf_yaw;

int sampleFreq;
// #define sampleFreq	100.0f // sample frequency in Hz
#define G_2_MPSS 1.0
#define betaDef		0.1f // 2 * proportional gain
float beta = betaDef; // 2 * proportional gain (Kp)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

bool valid_first_tick_flag; ///
double left_ticks, right_ticks;
double left_ticks_prev, right_ticks_prev;
double delta_left_ticks, delta_right_ticks;

double self_x=0;
double self_y=0;
double self_th=0;
double base_width, ticks_per_meter;
ros::NodeHandle *private_n;
// IMU algorithm update

// Fast inverse square-root
float invSqrt(float x)
{
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

	/*
	* get Euler angles
	* aerospace sequence, to obtain sensor attitude:
	* 1. rotate around sensor Z plane by yaw
	* 2. rotate around sensor Y plane by pitch
	* 3. rotate around sensor X plane by roll
	*/
void quat2eulerOK(void) {	//float *roll, float *pitch, float *yaw

		vf_yaw =  atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))* 180.0/M_PI;  // psi 'yaw
		vf_pitch = asin(2 * (q0 * q2 - q3 * q1))* 180.0/M_PI; //theta = pitch
		vf_roll =  (atan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3) ) * 180.0/M_PI *(-1)); // phi 'roll
}

//  Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
void getEulerErr(float * angles) {
  float q[4]; // quaternion
  // getQ(q);
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
  angles[1] = asin(2 * q[0] * q[2] - 2 * q[1] * q[3] ) * 180/M_PI; // theta'OK!!
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// phi, theta, psi'

// https://en.wikipedia.org/wiki/Quaternion

void getYawPitchRoll(float * ypr) {
  float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  // getQ(q);

  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;
}


void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	// Normalise quaternion
}

//
int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "xnet_imuRaw_node" );
    ros::NodeHandle nh;
    private_n= new ros::NodeHandle("~");

    // ros::Subscriber ticks_sub = nh.subscribe("/encoder", 20, ticksLR_callback);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/quatIMU", 20);
    ros::Publisher pubRazorImu = nh.advertise<robbase_msg::RazorImu>("/rpyTFimu", 1);
    ros::Publisher pubRazorEulerImu = nh.advertise<robbase_msg::RazorImu>("/rpyEulerImu", 1);

    ros::Publisher pubResiduAccel = nh.advertise<robbase_msg::RazorImu>("/xyzGravAccel", 1);

    RTQuaternion rt_rotatedGravity;
    RTQuaternion rt_fusedConjugate;
    RTQuaternion rt_gravity;
    rt_gravity.setScalar(0);
    rt_gravity.setX(0);
    rt_gravity.setY(0);
    rt_gravity.setZ(1);

    valid_first_tick_flag = false; ///
    // nh.param("base_width", base_width, 0.5);
    double gyro_bias;
    if(!private_n->getParam("gyro_bias", gyro_bias)) {
        ROS_WARN("No gyro_bias provided - default: 0");
        gyro_bias = 0.0;
    }

	std::string imu_frame_id_;
    if(!private_n->getParam("imu_frame", imu_frame_id_)) {
        ROS_WARN("No imu_frame provided - default: imu");
        imu_frame_id_ = "imu";
    }

    int imuFreq;
    // nh.param("ticks_per_meter", ticks_per_meter,88000);
    if(!private_n->getParam("imuFreq", imuFreq)) {
        ROS_WARN("No imuFreq provided - default: 24298");
        imuFreq = 50;
    }
    sampleFreq = imuFreq;

    geometry_msgs::TransformStamped imu_transform_msg;
    tf::TransformBroadcaster imu_tf_broadcaster;
    ros::Rate loop_rate(20);
    ROS_INFO("Node base_imuetry started");

    last_time = ros::Time::now();

    while (ros::ok()) {
        double dx, dr, dist, dtheta, d_left, d_right;
        double x, y;
        current_time = ros::Time::now();
	uint64_t timestamp_ga = RTMath::currentUSecsSinceEpoch();

        double elapsed_dt = (current_time - last_time).toSec();
	 xdriver_get("imuRaw", &imuRaw_struct, 24);
        // 2ex'scale;
        vf_gx = (float) imuRaw_struct.gx * 0.001;
        vf_gy = (float) imuRaw_struct.gy * 0.001;
        vf_gz = (float) imuRaw_struct.gz * 0.001;
        vf_ax = (float) imuRaw_struct.ax * 0.001;
        vf_ay = (float) imuRaw_struct.ay * 0.001;
        vf_az = (float) imuRaw_struct.az * 0.001;
        // vs' call :: madgwick 'AHRS 'see' .c'.h 'github'a= sivertism/madgwicks_AHRS_algorithm_c--sivertism _y16m2git
        // MadgwickAHRSupdateIMU(vf_gx, vf_gy, vf_gz, vf_ax, vf_ay, vf_az);
	
	rt_accel.setX(vf_ax); 
	rt_accel.setY(vf_ay);
	rt_accel.setZ(vf_az);
	rt_gyro.setX(vf_gx); 
	rt_gyro.setX(vf_gy); 
	rt_gyro.setX(vf_gz);
	rt_fusion.newIMUData(rt_gyro, rt_accel, timestamp_ga);

	// git' RTIMULib/RTIMULib.git ': RTVector3 RTFusion::getAccelResiduals()
	// show: Accelerometer residuals (g):
	//// rt_residuAccel = rt_fusion.getAccelResiduals(rt_accel);

	// ie. see: RTIMULib-Arduino.git: ArduinoAccel.ino 'Ln80s
	// adjust the measured accel and change the signs to make sense
        //  do gravity rotation and subtraction
        
        // RTQuaternion rt_fusedConjugate, rt_rotatedGravity;
        rt_fusedConjugate = rt_fusion.getFusionQPose().conjugate();
        // do the rotation - takes two steps
        rt_rotatedGravity = rt_fusedConjugate * ( rt_gravity * rt_fusion.getFusionQPose() );

        rt_residuAccel.setX( (vf_ax - rt_rotatedGravity.x()) *(-1.0) );
        rt_residuAccel.setY( (vf_ay - rt_rotatedGravity.y()) *(-1.0) );
        rt_residuAccel.setZ( (vf_ay - rt_rotatedGravity.y()) *(-1.0) ); //

	robbase_msg::RazorImu rz_residuAccel;
        rz_residuAccel.roll = rt_residuAccel.x();
        rz_residuAccel.pitch = rt_residuAccel.y() ;
        rz_residuAccel.yaw = rt_residuAccel.z();
	// rz_vf.header.frame_id = " ";
        rz_residuAccel.header.stamp = current_time;
        pubResiduAccel.publish(rz_residuAccel);
        // RTMath::display("residuAccel'ax'ay'az:", rt_residuAccel);

	// ros::Time current_time = ros::Time::now();

        // publish the /imu topic
        sensor_msgs::Imu imuMsg;
        robbase_msg::RazorImu rz;
        robbase_msg::RazorImu rz_vf;
        imuMsg.header.stamp = current_time;
        imuMsg.header.frame_id = imu_frame_id_; // "imu";

        //set the velocity
        imuMsg.angular_velocity.x=vf_gx;
        imuMsg.angular_velocity.y=vf_gy;
        imuMsg.angular_velocity.z=vf_gz;

        imuMsg.linear_acceleration.x=vf_ax * G_2_MPSS;
        imuMsg.linear_acceleration.y=vf_ay * G_2_MPSS;
        imuMsg.linear_acceleration.z=vf_az * G_2_MPSS;

       // 2e' madgwick' Quaternion q0'q1'q2'q3
       double roll, pitch, yaw;
	// We use a quaternion created from yaw
       tf::Quaternion imu_quat = tf::Quaternion(q0, q1, q2, q3 );
       tf::Matrix3x3(imu_quat).getRPY(roll, pitch, yaw);
      // tf Message
      tf::Transform transform_tf;
      transform_tf.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
      transform_tf.setRotation(imu_quat);

	// publishing 'tf: imu/ base_link
	// imu_tf_broadcaster.sendTransform(imu_transform_msg);
	// imu_tf_broadcaster.sendTransform( (0.0, 0.0, 0.0), (q0, q1, q2, q3), current_time, "base_link", imu_frame_id_);
        imu_tf_broadcaster.sendTransform( tf::StampedTransform(transform_tf, ros::Time::now(), "base_link", imu_frame_id_) );
        // current_time = ros::Time::now();

	// imu_transform_msg.header.frame_id = "imu"; // imu_frame_id_;
	// imu_transform_msg.header.child_frame_id = "base_link";
	// imu_transform_msg.header.stamp = current_time;

        // imu message
        // imuMsg.orientation = imu_quat;
        imuMsg.orientation.x=q0; //TODO orientation
        imuMsg.orientation.y=q1;
        imuMsg.orientation.z=q2;
        imuMsg.orientation.w=q3;

        quat2eulerOK();
        rz_vf.header.stamp = current_time;
        // rz_vf.header.frame_id = " ";
        rz_vf.roll = vf_roll;
        rz_vf.pitch = vf_pitch;
        rz_vf.yaw = vf_yaw;

        rz.header.stamp = current_time;
        // rz.header.frame_id = " ";
        rz.yaw = (float) yaw;
        rz.pitch = (float) pitch;
        rz.roll = (float) roll;
		// publish the /imu topic
        imu_pub.publish(imuMsg);
        pubRazorEulerImu.publish(rz_vf);
        pubRazorImu.publish(rz);

        last_time = current_time;

        loop_rate.sleep();
    }// end.while _ ros::ok '
}// end.main

