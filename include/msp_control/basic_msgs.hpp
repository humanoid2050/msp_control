#ifndef BASIC_MSGS_HPP
#define BASIC_MSGS_HPP


//state messages to host

//-----------------MSP MultiWii---------------
//FC identity -> custom
//sensor states -> custom
//raw accel, gyro, mag 
//raw rc input -> custom
//roll, pitch, heading 
//altitude, z_speed
//analog batt, power, signal, rssi
//waypoint
//navigation status
//navigation config?



//position xyz, orientation wxyz
#include <geometry_msgs/PoseStamped.h>
//quaternion orientation, rpy rate, xyz accel
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>


//control messages from host

//inject rc
//inject gps
//inject heading
//set waypoint



#endif
