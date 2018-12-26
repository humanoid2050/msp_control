
#include "ros/ros.h"

#include "sensor_msgs/Joy.h"
#include "utilities/platform_control.h"
#include "utilities/BoolStamped.h"
#include "msp_control/FlightMode.h"

class Teleop
{
public:
    enum MODE {ANGLE, NAV_HOLD, RTH};
    
    Teleop() : activate_(false), hold_altitude_(false), mode_(ANGLE)
    {}
    
    void generateCommands(const sensor_msgs::Joy& joyMsg)
    {
        
        
        if (joyMsg.buttons[activate_channel_]) {
            activate_ = true;
        }
        if (joyMsg.buttons[deactivate_channel_]) {
            activate_ = false;
        }
        
        utilities::platform_control control;
        control.commandType = utilities::platform_control::TYPE_QUAD_SIMPLE;
        control.value[0] = joyMsg.axes[pitch_channel_]; // pitch / x motion
        control.value[1] = joyMsg.axes[roll_channel_]; // roll / y motion 
        control.value[2] = joyMsg.axes[throttle_channel_]; // thrust / z motion
        control.value[3] = joyMsg.axes[yaw_channel_]; // yaw / z rotation
        command_pub_.publish(control);
        
        //configure flight mode (ANGLE/NAV_HOLD/RTH, ALT_HOLD, ARM)
        msp_control::FlightMode flight_mode;
        switch ( mode_) {
        case ANGLE:
            flight_mode.primary_mode = msp_control::FlightMode::ANGLE;
            break;
        case NAV_HOLD:
            flight_mode.primary_mode = msp_control::FlightMode::NAV_ALTHOLD;
            break;
        case RTH:
            flight_mode.primary_mode = msp_control::FlightMode::NAV_RTH;
            break;
        }
        
        if (hold_altitude_) flight_mode.secondary_mode = msp_control::FlightMode::NAV_ALTHOLD;
        if (activate_) flight_mode.modifier = msp_control::FlightMode::ARM;
        flight_mode_pub_.publish(flight_mode);
        
        //if activated, set MSP control source (else SBUS)
        utilities::BoolStamped msp_control;
        msp_control.header = std_msgs::Header();
        msp_control.value = activate_;
        control_source_pub_.publish(msp_control);
        
    }
    
    
    int throttle_channel_;
    int roll_channel_;
    int pitch_channel_;
    int yaw_channel_; 
    
    int activate_channel_;
    int deactivate_channel_;
    
    
    bool activate_; // arm and SBUS/MSP
    bool hold_altitude_;
    MODE mode_;
    
    
    ros::Publisher command_pub_;
    ros::Publisher flight_mode_pub_;
    ros::Publisher control_source_pub_;
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle n;

	Teleop teleop;
    
    ros::Subscriber joy_event = n.subscribe("joy", 5, &Teleop::generateCommands, &teleop);
    
    teleop.command_pub_ = n.advertise<utilities::platform_control>("control",10);
    teleop.flight_mode_pub_ = n.advertise<utilities::platform_control>("flight_mode",10);
    teleop.control_source_pub_ = n.advertise<utilities::platform_control>("control_source",10);

    ROS_INFO("STARTED TELEOP SPIN");
    ros::spin();

}
