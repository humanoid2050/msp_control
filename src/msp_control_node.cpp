//#include "msp_control/basic_msgs_interface.hpp"

#include "ros/ros.h"

#include "msp/FlightController.hpp"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "utilities/platform_control.h"
#include "utilities/Float64Stamped.h"
#include "utilities/BoolStamped.h"
#include "msp_control/FlightMode.h"

#include <iostream>
#include <math.h>

using msp_control::FlightMode;
using utilities::platform_control;

class App {
public:

    enum SOURCE {MSP, SBUS};
    enum MODE {ANGLE, NAV_HOLD, RTH};
    
    enum rxReceiverType_e {
        RX_TYPE_NONE        = 0,
        RX_TYPE_PWM         = 1,
        RX_TYPE_PPM         = 2,
        RX_TYPE_SERIAL      = 3,
        RX_TYPE_MSP         = 4,
        RX_TYPE_SPI         = 5,
        RX_TYPE_UIB         = 6
    } ;

    App(ros::NodeHandle& n) : node_(n), source_(SBUS), mode_(ANGLE), armed_(false), altitude_hold_(false)
    {
        imu_pub = node_.advertise<sensor_msgs::Imu>("imu", 10);
        mag_pub = node_.advertise<sensor_msgs::MagneticField>("mag", 10);
        alt_pub = node_.advertise<sensor_msgs::MagneticField>("altitude", 10);
        nav_pub = node_.advertise<sensor_msgs::MagneticField>("nav", 10);
    }

    void onStatus(msp::msg::Status& status) {
        std::cout<<status;
    }

    void onImu(msp::msg::RawImu& imu_raw) {
        std::cout<< imu_raw;
        
        msp::msg::ScaledImu imu_scaled(imu_raw, 9.80665f/512.0, 1.0/4.096, 0.92f/10.0f);
        
        sensor_msgs::Imu imu;
        imu.header = std_msgs::Header();
        imu.orientation = tf2::toMsg(orientation_);
        imu.angular_velocity.x = imu_scaled.gyro[0]();
        imu.angular_velocity.y = imu_scaled.gyro[1]();
        imu.angular_velocity.z = imu_scaled.gyro[2]();
        imu.linear_acceleration.x = imu_scaled.acc[0]();
        imu.linear_acceleration.y = imu_scaled.acc[0]();
        imu.linear_acceleration.z = imu_scaled.acc[0]();
        
        imu_pub.publish(imu);
    
        sensor_msgs::MagneticField mag;
        mag.header = std_msgs::Header();
        mag.magnetic_field.x = imu_scaled.mag[0]();
        mag.magnetic_field.y = imu_scaled.mag[1]();
        mag.magnetic_field.z = imu_scaled.mag[2]();
        
        mag_pub.publish(mag);

    }

    void onGps(msp::msg::RawGPS& gps_raw) {
        std::cout<< gps_raw;  
        
        sensor_msgs::NavSatFix fix; 
        fix.header = std_msgs::Header();
        fix.status.status = gps_raw.fix();
        fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        fix.latitude = gps_raw.lat(); 
        fix.longitude = gps_raw.lon();
        fix.altitude = gps_raw.altitude();
        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        
        nav_pub.publish(fix);
    }
    
    void onCompGps(msp::msg::CompGPS& home_offset) {
        std::cout<< home_offset;
    }
    
    void onGpsStatistics(msp::msg::GpsStatistics& gps_stats) {
        std::cout<< gps_stats;   
    }
    

    void onAttitude(msp::msg::Attitude& attitude) {
        std::cout<<attitude;
        orientation_.setRPY((attitude.roll())*M_PIl/180.d, (attitude.pitch())*M_PIl/180.d, (attitude.yaw())*M_PIl/180.d);
    }

    void onAltitude(msp::msg::Altitude& altitude) {
        std::cout<<altitude;
        
        utilities::Float64Stamped alt;
        alt.header = std_msgs::Header();
        alt.value = altitude.altitude();
        alt_pub.publish(alt);
    }
    
    void arm()
    {
        armed_ = true;
    }
    
    void disarm()
    {
        armed_ = false;
    }
    
    void setAltitudeHold(bool on)
    {
        altitude_hold_ = on;
    }
    
    void setSource(SOURCE src)
    {
        source_ = src;
    }
    
    void setMode(MODE mode)
    {
        mode_ = mode;
    }
    
    std::vector<uint16_t> inputControl(const platform_control& control)
    {
        if (control.commandType != platform_control::TYPE_QUAD_SIMPLE) return std::vector<uint16_t>();
        
        
        std::vector<uint16_t> cmds;
        cmds.emplace_back((control.value[1]+1)*1500);
        cmds.emplace_back((control.value[0]+1)*1500);
        cmds.emplace_back((control.value[3]+1)*1500);
        cmds.emplace_back((control.value[2]+1)*1500);
        if (!armed_) cmds.emplace_back(1000);
        else if (altitude_hold_) cmds.emplace_back(2000);
        else cmds.emplace_back(1500);
        
        switch (mode_) {
        case ANGLE:
            cmds.emplace_back(1000);
            break;
        case NAV_HOLD:
            cmds.emplace_back(1500);
            break;
        case RTH:
            cmds.emplace_back(2000);
            break;
        }
        return cmds;
    }
    
private:
    ros::NodeHandle& node_;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher alt_pub;
    ros::Publisher nav_pub;
    
    tf2::Quaternion orientation_;
    
    SOURCE source_;
    MODE mode_;
    bool armed_;
    bool altitude_hold_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle n;

	std::string serial_dev;

    n.param<std::string>("serial_dev", serial_dev, "/dev/ttyACM0");
    
    const std::string device = (argc>1) ? std::string(argv[1]) : serial_dev;
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    fcu::FlightController fcu(device, baudrate);

    // wait for connection
    fcu.initialise();

    App app(n);
    std::cout << "made app" << std::endl;
    
    fcu.subscribe(&App::onStatus, &app, 0.1);
    fcu.subscribe(&App::onImu, &app, 0.1);
    fcu.subscribe(&App::onGps, &app, 0.1);
    fcu.subscribe(&App::onCompGps, &app, 1);
    fcu.subscribe(&App::onGpsStatistics, &app, 1);
    fcu.subscribe(&App::onAttitude, &app, 0.1);
    fcu.subscribe(&App::onAltitude, &app, 0.1);
    
    //handle control messages
    boost::function<void (const platform_control&)> setRcValues = 
        [&] (const platform_control& message) { fcu.setRc(app.inputControl(message)); };
    ros::Subscriber control_sub = n.subscribe<platform_control>("control", 1, setRcValues);
    

    //handle flight mode messages
    boost::function<void (const FlightMode&)> setMode =
        [&] (const FlightMode& flight_mode) {
            if ( flight_mode.primary_mode == FlightMode::ANGLE ) app.setMode(App::ANGLE); 
            else if ( flight_mode.primary_mode == FlightMode::NAV_POSHOLD ) app.setMode(App::NAV_HOLD); 
            else if ( flight_mode.primary_mode == FlightMode::NAV_RTH ) app.setMode(App::RTH); 
            
            app.setAltitudeHold(flight_mode.secondary_mode & FlightMode::NAV_ALTHOLD);
            
            if ( flight_mode.modifier & FlightMode::ARM ) app.arm();
            else app.disarm();
            
        };
    
    ros::Subscriber flight_mode_sub = n.subscribe<FlightMode>("flight_mode", 1, setMode );
    
    //handle source selection messages
    boost::function<void (const App::SOURCE&)> setSource = 
        [&] (const App::SOURCE& source) {    
            msp::msg::RxConfig currentRxConfig(msp::FirmwareVariant::INAV);
            fcu.request(currentRxConfig);
            msp::msg::SetRxConfig setRxConfig(msp::FirmwareVariant::INAV);
            static_cast<msp::msg::RxConfigSettings&>(setRxConfig) = static_cast<msp::msg::RxConfigSettings&>(currentRxConfig);
            setRxConfig.serialrx_provider = (source == App::MSP ? App::RX_TYPE_MSP : App::RX_TYPE_SERIAL);
            fcu.respond(setRxConfig);
        };
    boost::function<void (const utilities::BoolStamped&)> binarySetSource =
        [&] (const utilities::BoolStamped& use_msp) { setSource( use_msp.value ? App::MSP : App::SBUS ); };
    ros::Subscriber control_source_sub = n.subscribe<utilities::BoolStamped>("control_source", 1 , binarySetSource);

    ROS_INFO("STARTED MSP NODE SPIN");
    
    app.setMode(App::ANGLE);
    app.setSource(App::MSP);
    app.arm();
    
    ros::spin();
    
    app.disarm();
    app.setSource(App::SBUS);
    
    
}
