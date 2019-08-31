//#include "msp_control/basic_msgs_interface.hpp"

#include "ros/ros.h"

#include "msp/FCFactory.hpp"
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
#include "msp_control/ModeMap.hpp"

#include <iostream>
#include <math.h>

#include <signal.h>


using msp_control::FlightMode;
using utilities::platform_control;

class App {
public:

    App(ros::NodeHandle& n) : node_(n)//, armed_(false), altitude_hold_(false) // mode_(ANGLE), 
    {
        imu_pub = node_.advertise<sensor_msgs::Imu>("imu", 10);
        mag_pub = node_.advertise<sensor_msgs::MagneticField>("mag", 10);
        alt_pub = node_.advertise<utilities::Float64Stamped>("altitude", 10);
        nav_pub = node_.advertise<sensor_msgs::NavSatFix>("nav", 10);
    }

    void onStatus(const msp::msg::InavStatus<>& status) {
        //std::cout << status;
        std::string arming_flags = msp::armingFlagToString(status.arming_flags());
        
        if (arming_flags != arming_flags_) {
            arming_flags_ = arming_flags;
            std::cout << arming_flags << std::endl;
        }
        
    }
    
    void onRc(const msp::msg::Rc<>& rc) {
        //std::cout << rc;
    }

    void onImu(const msp::msg::RawImu<>& imu_raw) {
        //std::cout<< imu_raw;
        
        msp::msg::ImuSI<> imu_scaled(imu_raw, 512, M_PIl/180.0/4.096, 0.92f/10.0f, 9.80665f);
        
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

    void onGps(const msp::msg::RawGPS<>& gps_raw) {
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
    
    void onCompGps(const msp::msg::CompGPS<>& home_offset) {
        //std::cout<< home_offset;
    }
    
    void onGpsStatistics(const msp::msg::GpsStatistics<>& gps_stats) {
        //std::cout<< gps_stats;   
    }
    

    void onAttitude(const msp::msg::Attitude<>& attitude) {
        //std::cout<<attitude;
        orientation_.setRPY((attitude.roll())*M_PIl/180.d, (attitude.pitch())*M_PIl/180.d, (attitude.yaw())*M_PIl/180.d);
    }

    void onAltitude(const msp::msg::Altitude<>& altitude) {
        //std::cout<<altitude;
        
        utilities::Float64Stamped alt;
        alt.header = std_msgs::Header();
        alt.value = altitude.altitude();
        alt_pub.publish(alt);
    }
private:
    ros::NodeHandle& node_;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher alt_pub;
    ros::Publisher nav_pub;
    
    std::string arming_flags_;
    
    tf2::Quaternion orientation_;
    
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "msp_control_node");
    ros::NodeHandle n("~");
    
	std::string serial_dev;

    n.param("serial_dev", serial_dev, std::string("/dev/ttyACM0"));
    
    const std::string device = (argc>1) ? std::string(argv[1]) : serial_dev;
    const size_t baudrate = (argc>2) ? std::stoul(argv[2]) : 115200;

    auto fcu = msp::FlightControllerFactory::create(device,baudrate);
    
    fcu->start(device, baudrate);
    std::cout << "INITIALIZED" <<std::endl;

    App app(n);
    
    fcu->subscribe(&App::onStatus, &app, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    fcu->subscribe(&App::onImu, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    fcu->subscribe(&App::onGps, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //fcu->subscribe(&App::onCompGps, &app, 0.1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //fcu->subscribe(&App::onGpsStatistics, &app, 0.1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    fcu->subscribe(&App::onAttitude, &app, 0.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    fcu->subscribe(&App::onAltitude, &app, 0.1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //fcu->subscribe(&App::onRc, &app, 0.1);
    
    //handle control messages
    boost::function<void (const platform_control&)> setRcValues = 
        [&] (const platform_control& message) { 
            if (message.commandType != platform_control::TYPE_QUAD_SIMPLE) return;

            std::array<double,4> rpyt = {message.value[1],message.value[0],message.value[3],message.value[2]};
            fcu->setRPYT(rpyt);
        };
    ros::Subscriber control_sub = n.subscribe<platform_control>("control", 1, setRcValues);
    
    
    //handle flight mode messages
    boost::function<void (const FlightMode&)> setMode =
        [&] (const FlightMode& flight_mode) {
            
            std::set<std::string> set_modes;
            for (const auto& mode : flight_mode.set) {
                set_modes.emplace(ModeMap.at(mode));
            }
            fcu->setMspModes(set_modes);
            
            std::set<std::string> add_modes;
            for (const auto& mode : flight_mode.set) {
                add_modes.emplace(ModeMap.at(mode));
            }
            std::set<std::string> remove_modes;
            for (const auto& mode : flight_mode.set) {
                remove_modes.emplace(ModeMap.at(mode));
            }
            
            fcu->updateMspModes(add_modes, remove_modes);
        };
    ros::Subscriber flight_mode_sub = n.subscribe<FlightMode>("flight_mode", 1, setMode );
    
    
    //handle source selection messages
    boost::function<void (const utilities::BoolStamped&)> binarySetSource =
        [&] (const utilities::BoolStamped& use_msp) { 
            
            fcu->setRadioControlType( use_msp.value ? msp::RadioControlType::MSP : msp::RadioControlType::SERIAL ); 

        };
    ros::Subscriber control_source_sub = n.subscribe<utilities::BoolStamped>("control_source", 10 , binarySetSource);
    
    
    std::cout << "STARTED MSP NODE SPIN" << std::endl;
    fcu->setLoggingLevel(msp::LoggingLevel::INFO);
    
    ros::spin();
    std::cout << "LOOP FINISHED" <<std::endl;
    fcu->stop();
    
}
