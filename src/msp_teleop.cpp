
#include "ros/ros.h"

#include "sensor_msgs/Joy.h"
#include "utilities/platform_control.h"
#include "utilities/BoolStamped.h"
#include "msp_control/FlightMode.h"

#include <iostream>

class Teleop
{
public:
    enum MODE {ANGLE, NAV_HOLD, RTH};
    
    Teleop() : activate_(false), arm_(false), hold_altitude_(false), mode_(ANGLE)
    {}
    
    void interpretJoystick(const sensor_msgs::Joy& joyMsg)
    {
        
        if (joyMsg.buttons[activate_channel_]) { activate_ = true; }
        if (joyMsg.buttons[deactivate_channel_]) { activate_ = false; }
        
        std::cout << arm_channel_ << " " << disarm_channel_ << " " << alt_hold_channel_ << " " << no_alt_hold_channel_ << std::endl;
        std::cout << joyMsg.buttons[arm_channel_] << " " << joyMsg.buttons[disarm_channel_] << " " << joyMsg.buttons[alt_hold_channel_] << " " << joyMsg.buttons[no_alt_hold_channel_] << std::endl;
        if (joyMsg.buttons[arm_channel_]) { arm_ = true; }
        if (joyMsg.buttons[disarm_channel_]) { arm_ = false; }
        if (joyMsg.buttons[alt_hold_channel_]) { hold_altitude_ = true; }
        if (joyMsg.buttons[no_alt_hold_channel_]) { hold_altitude_ = false; }
        
        std::cout << rth_channel_ << " " << nav_hold_channel_ << " " << angle_channel_ << std::endl;
        std::cout << joyMsg.buttons[rth_channel_] << " " << joyMsg.buttons[nav_hold_channel_] << " " << joyMsg.buttons[angle_channel_] << std::endl;
        if (joyMsg.buttons[rth_channel_]) { mode_ = RTH; }
        if (joyMsg.buttons[nav_hold_channel_]) { mode_ = NAV_HOLD; }
        if (joyMsg.buttons[angle_channel_]) { mode_ = ANGLE; }
        
        utilities::platform_control control;
        control.header = std_msgs::Header();
        control.commandType = utilities::platform_control::TYPE_QUAD_SIMPLE;
        control.value[0] = invert_pitch_ ? -joyMsg.axes[pitch_channel_] : joyMsg.axes[pitch_channel_]; // pitch / x motion
        control.value[1] = invert_roll_ ? -joyMsg.axes[roll_channel_] : joyMsg.axes[roll_channel_]; // roll / y motion 
        control.value[2] = invert_throttle_ ? -joyMsg.axes[throttle_channel_] : joyMsg.axes[throttle_channel_]; // thrust / z motion
        control.value[3] = invert_yaw_ ? -joyMsg.axes[yaw_channel_] : joyMsg.axes[yaw_channel_]; // yaw / z rotation
        command_pub_.publish(control);
        
        //configure flight mode (ANGLE/NAV_HOLD/RTH, ALT_HOLD, ARM)
        msp_control::FlightMode flight_mode;
        flight_mode.header = std_msgs::Header();
        switch ( mode_) {
        case ANGLE:
            flight_mode.primary_mode = msp_control::FlightMode::ANGLE;
            break;
        case NAV_HOLD:
            flight_mode.primary_mode = msp_control::FlightMode::NAV_POSHOLD;
            break;
        case RTH:
            flight_mode.primary_mode = msp_control::FlightMode::NAV_RTH;
            break;
        }
        
        if (hold_altitude_) flight_mode.secondary_mode = msp_control::FlightMode::NAV_ALTHOLD;
        if (arm_) flight_mode.modifier = msp_control::FlightMode::ARM;
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
    
    bool invert_throttle_;
    bool invert_roll_;
    bool invert_pitch_;
    bool invert_yaw_;
    
    int activate_channel_;
    int deactivate_channel_;
    
    int arm_channel_;
    int disarm_channel_;
    
    int alt_hold_channel_;
    int no_alt_hold_channel_;
    
    int angle_channel_;
    int nav_hold_channel_;
    int rth_channel_;
    
    //status flags
    bool activate_; // SBUS/MSP
    bool arm_;
    bool hold_altitude_;
    MODE mode_;
    
    
    ros::Publisher command_pub_;
    ros::Publisher flight_mode_pub_;
    ros::Publisher control_source_pub_;
    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_node");
    ros::NodeHandle n("~");

	Teleop teleop;
    //buttons
    n.param("activate_channel", teleop.activate_channel_, 5);
    n.param("deactivate_channel", teleop.deactivate_channel_, 4);
    
    n.param("arm_channel", teleop.arm_channel_, 1);
    n.param("disarm_channel", teleop.disarm_channel_, 0);
    
    n.param("alt_hold_channel", teleop.alt_hold_channel_, 2);
    n.param("no_alt_hold_channel", teleop.no_alt_hold_channel_, 3);
    
    n.param("angle_channel", teleop.angle_channel_, 14);
    n.param("nav_hold_channel", teleop.nav_hold_channel_, 15);
    n.param("rth_channel", teleop.rth_channel_, 13);
    //axes
    n.param("throttle_channel", teleop.throttle_channel_, 1);
    n.param("roll_channel", teleop.roll_channel_, 3);
    n.param("pitch_channel", teleop.pitch_channel_, 4);
    n.param("yaw_channel", teleop.yaw_channel_, 0);
    //inversions
    n.param("invert_throttle", teleop.invert_throttle_, false);
    n.param("invert_roll", teleop.invert_roll_, false);
    n.param("invert_pitch", teleop.invert_pitch_, false);
    n.param("invert_yaw", teleop.invert_yaw_, false);
    
    
    teleop.command_pub_ = n.advertise<utilities::platform_control>("/control",10);
    teleop.flight_mode_pub_ = n.advertise<msp_control::FlightMode>("/flight_mode",10);
    teleop.control_source_pub_ = n.advertise<utilities::BoolStamped>("/control_source",10);
    
    ros::Subscriber joy_event = n.subscribe("/joy", 5, &Teleop::interpretJoystick, &teleop);

    ROS_INFO("STARTED TELEOP SPIN");
    ros::spin();

}
