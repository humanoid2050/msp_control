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

#include "utilities/platform_control.h"
#include "utilities/Float64Stamped.h"
#include "utilities/BoolStamped.h"
#include "msp_control/FlightMode.h"

#include <iostream>
#include <math.h>

#include <signal.h>

void mySigintHandler(int sig)
{
    std::cout << "signal caught" <<std::endl;
    ros::shutdown();
    std::cout << "sent shutdown" <<std::endl;
}


using msp_control::FlightMode;
using utilities::platform_control;

class App {
public:

    //enum SOURCE {UNSET, MSP, SBUS, OTHER};
    //enum FLIGHTMODE {ANGLE, NAV_HOLD, RTH};
    /*
    enum rxReceiverType_e {
        RX_TYPE_NONE        = 0,
        RX_TYPE_PWM         = 1,
        RX_TYPE_PPM         = 2,
        RX_TYPE_SERIAL      = 3,
        RX_TYPE_MSP         = 4,
        RX_TYPE_SPI         = 5,
        RX_TYPE_UIB         = 6
    } ;
    */
    App(ros::NodeHandle& n) : node_(n)//, armed_(false), altitude_hold_(false) // mode_(ANGLE), 
    {
        imu_pub = node_.advertise<sensor_msgs::Imu>("imu", 10);
        mag_pub = node_.advertise<sensor_msgs::MagneticField>("mag", 10);
        alt_pub = node_.advertise<utilities::Float64Stamped>("altitude", 10);
        nav_pub = node_.advertise<sensor_msgs::NavSatFix>("nav", 10);
    }

    void onStatus(msp::msg::InavStatus& status) {
        std::cout<<status;
    }
    
    void onRc(msp::msg::Rc& rc) {
        std::cout << rc;
    }

    void onImu(msp::msg::RawImu& imu_raw) {
        //std::cout<< imu_raw;
        
        msp::msg::ScaledImu imu_scaled(imu_raw, 9.80665f/512.0, M_PIl/180.0/4.096, 0.92f/10.0f);
        
        sensor_msgs::Imu imu;
        imu.header = std_msgs::Header();
        
        imu.orientation.w = orientation_.getW();
        imu.orientation.x = orientation_.getX();
        imu.orientation.y = orientation_.getY();
        imu.orientation.z = orientation_.getZ();
        
        imu.angular_velocity.x = imu_scaled.gyro[0]();
        imu.angular_velocity.y = imu_scaled.gyro[1]();
        imu.angular_velocity.z = imu_scaled.gyro[2]();
        imu.linear_acceleration.x = imu_scaled.acc[0]();
        imu.linear_acceleration.y = imu_scaled.acc[1]();
        imu.linear_acceleration.z = imu_scaled.acc[2]();
        
        imu_pub.publish(imu);
    
        sensor_msgs::MagneticField mag;
        mag.header = std_msgs::Header();
        mag.magnetic_field.x = imu_scaled.mag[0]();
        mag.magnetic_field.y = imu_scaled.mag[1]();
        mag.magnetic_field.z = imu_scaled.mag[2]();
        
        mag_pub.publish(mag);

    }

    void onGps(msp::msg::RawGPS& gps_raw) {
        //std::cout<< gps_raw;  
        
        sensor_msgs::NavSatFix fix; 
        fix.header = std_msgs::Header();
        fix.status.status = (gps_raw.fix() > 0 ? sensor_msgs::NavSatStatus::STATUS_FIX : sensor_msgs::NavSatStatus::STATUS_NO_FIX) ;
        fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        fix.latitude = gps_raw.lat(); 
        fix.longitude = gps_raw.lon();
        fix.altitude = gps_raw.altitude();
        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        
        nav_pub.publish(fix);
    }
    
    void onCompGps(msp::msg::CompGPS& home_offset) {
        //std::cout<< home_offset;
    }
    
    void onGpsStatistics(msp::msg::GpsStatistics& gps_stats) {
        //std::cout<< gps_stats;   
    }
    

    void onAttitude(msp::msg::Attitude& attitude) {
        //std::cout<<attitude;
        orientation_.setRPY((attitude.roll())*M_PIl/180.d, (attitude.pitch())*M_PIl/180.d, (attitude.yaw())*M_PIl/180.d);
    }

    void onAltitude(msp::msg::Altitude& altitude) {
        //std::cout<<altitude;
        
        utilities::Float64Stamped alt;
        alt.header = std_msgs::Header();
        alt.value = altitude.altitude();
        alt_pub.publish(alt);
    }
    /*
    void arm()
    {
        armed_ = true;
    }
    
    void disarm()
    {
        armed_ = false;
    }
    
    bool isArmed()
    {
        return armed_;
    }
    
    void setAltitudeHold(bool on)
    {
        altitude_hold_ = on;
    }
    
    bool getAltitudeHold()
    {
        return altitude_hold_;
    }
    
    
    void setSource(fcu::ControlSource src)
    {
        source_ = src;
    }
    
    fcu::ControlSource getSource()
    {
        return source_;
    }
    
    void setMode(FLIGHTMODE mode)
    {
        mode_ = mode;
    }
    
    FLIGHTMODE getMode()
    {
        return mode_;
    }
    
    std::vector<uint16_t> inputControl(const platform_control& control)
    {
        
        
        //std::cout << "std::vector<uint16_t> inputControl(const platform_control& control)" << std::endl;
        if (control.commandType != platform_control::TYPE_QUAD_SIMPLE) return std::vector<uint16_t>();
        
        std::vector<uint16_t> cmds(18,1000);
        //manually remapping from EATR (platform control sequence) to TAER (american RC)
        
        cmds[0] = (control.value[2]*500)+1500;
        cmds[1] = (control.value[1]*500)+1500;
        cmds[2] = (control.value[0]*500)+1500;
        cmds[3] = (control.value[3]*500)+1500;
        
        if (!armed_) cmds[4] = 1000;
        else if (altitude_hold_) cmds[4] = 2000;
        else cmds[4] = 2000;
        
        switch (mode_) {
        case ANGLE:
            cmds[5] = 1000;
            break;
        case NAV_HOLD:
            cmds[5] = 1500;
            break;
        case RTH:
            cmds[5] = 2000;
            break;
        }
        
        return cmds;
    }
    */
    
private:
    ros::NodeHandle& node_;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher alt_pub;
    ros::Publisher nav_pub;
    
    tf2::Quaternion orientation_;
    
    //fcu::ControlSource source_;
    //FLIGHTMODE mode_;
    //bool armed_;
    //bool altitude_hold_;
    
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "msp_control_node");
    ros::NodeHandle n;
    
    

	std::string serial_dev;

    n.param<std::string>("serial_dev", serial_dev, "/dev/ttyACM0");
    
    const std::string device = (argc>1) ? std::string(argv[1]) : serial_dev;
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    fcu::FlightController fcu(device, baudrate);
    
    fcu.connect();
    std::cout << "INITIALIZED" <<std::endl;
    fcu.getControlSource();

    App app(n);
    
    fcu.subscribe(&App::onStatus, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    fcu.subscribe(&App::onImu, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    fcu.subscribe(&App::onGps, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    fcu.subscribe(&App::onCompGps, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    fcu.subscribe(&App::onGpsStatistics, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    fcu.subscribe(&App::onAttitude, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    fcu.subscribe(&App::onAltitude, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //fcu.subscribe(&App::onRc, &app, 0.1);
    
    //handle control messages
    boost::function<void (const platform_control&)> setRcValues = 
        [&] (const platform_control& message) { 
            if (message.commandType != platform_control::TYPE_QUAD_SIMPLE) return;
            std::cout << "-> platform control: " << std::endl;
            std::array<double,4> rpyt = {message.value[1],message.value[0],message.value[3],message.value[2]};
            fcu.setRPYT(rpyt);
            std::cout << "-> platform control finished" << std::endl;
        };
    ros::Subscriber control_sub = n.subscribe<platform_control>("control", 1, setRcValues);
    
    
    //handle flight mode messages
    boost::function<void (const FlightMode&)> setMode =
        [&] (const FlightMode& flight_mode) {
            std::cout << "-> set Flight Mode " << flight_mode.primary_mode << " " << flight_mode.secondary_mode << " " << flight_mode.modifier << std::endl;
            fcu::FlightMode current_fm;
            current_fm.primary = fcu::FlightMode::PRIMARY_MODE(flight_mode.primary_mode);
            current_fm.secondary = fcu::FlightMode::SECONDARY_MODE(flight_mode.secondary_mode);
            current_fm.modifier = fcu::FlightMode::MODIFIER(flight_mode.modifier);
            fcu.setFlightMode(current_fm);
            std::cout << "-> set Flight Mode finished" << std::endl;
        };
    ros::Subscriber flight_mode_sub = n.subscribe<FlightMode>("flight_mode", 1, setMode );
    
    
    //handle source selection messages
    boost::function<void (const utilities::BoolStamped&)> binarySetSource =
        [&] (const utilities::BoolStamped& use_msp) { 
            std::cout << "-> setting source from binary" << std::endl;
            fcu.setControlSource( use_msp.value ? fcu::ControlSource::MSP : fcu::ControlSource::SBUS ); 
            std::cout << "-> setting source from binary finished" << std::endl;
        };
    ros::Subscriber control_source_sub = n.subscribe<utilities::BoolStamped>("control_source", 10 , binarySetSource);
    
    
    std::cout << "STARTED MSP NODE SPIN" << std::endl;
    
    ros::spin();
    std::cout << "LOOP FINISHED" <<std::endl;
    fcu.disconnect();
    
}
