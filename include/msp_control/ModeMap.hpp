#ifndef MODE_MAP_HPP
#define MODE_MAP_HPP

#include "msp_control/FlightMode.h"

using namespace msp_control;

std::map<uint32_t,std::string> ModeMap = {
    
    {FlightMode::ARM, "ARM"},
    {FlightMode::ANGLE, "ANGLE"},
    {FlightMode::HORIZON, "HORIZON"},
    {FlightMode::NAVALTHOLD, "NAV ALTHOLD"},
    {FlightMode::HEADINGHOLD, "HEADING HOLD"},
    {FlightMode::HEADFREE, "HEADFREE"},
    {FlightMode::HEADADJ, "HEADADJ"},
    {FlightMode::CAMSTAB, "CAMSTAB"},
    {FlightMode::NAVRTH, "NAV RTH"},
    {FlightMode::NAVPOSHOLD, "NAV POSHOLD"},
    {FlightMode::MANUAL, "MANUAL"},
    {FlightMode::BEEPERON, "BEEPER"},
    {FlightMode::LEDLOW, "LEDLOW"},
    {FlightMode::LIGHTS, "LIGHTS"},
    {FlightMode::NAVLAUNCH, "NAV LAUNCH"},
    {FlightMode::OSD, "OSD SW"},
    {FlightMode::TELEMETRY, "TELEMETRY"},
    {FlightMode::BLACK, "BLACKBOX"},
    {FlightMode::FAILSAFE, "FAILSAFE"},
    {FlightMode::NAVWP, "NAV WP"},
    {FlightMode::AIRMODE, "AIR MODE"},
    {FlightMode::HOMERESET, "HOME RESET"},
    {FlightMode::GCSNAV, "GCS NAV"},
    {FlightMode::KILLSWITCH, "KILLSWITCH"},
    {FlightMode::SURFACE, "SURFACE"},
    {FlightMode::FLAPERON, "FLAPERON"},
    {FlightMode::TURNASSIST, "TURN ASSIST"},
    {FlightMode::AUTOTRIM, "SERVO AUTOTRIM"},
    {FlightMode::AUTOTUNE, "AUTO TUNE"},
    {FlightMode::CAMERA1, "CAMERA CONTROL 1"},
    {FlightMode::CAMERA2, "CAMERA CONTROL 2"},
    {FlightMode::CAMERA3, "CAMERA CONTROL 3"},
    {FlightMode::OSDALT1, "OSD ALT 1"},
    {FlightMode::OSDALT2, "OSD ALT 2"},
    {FlightMode::OSDALT3, "OSD ALT 3"},
    {FlightMode::NAVCRUISE, "NAV CRUISE"},
    {FlightMode::BRAKING, "MC BRAKING"},
    {FlightMode::USER1, "USER1"},
    {FlightMode::USER2, "USER2"},
    {FlightMode::FPVANGLEMIX, "FPV ANGLE MIX"},
    {FlightMode::LOITERDIRCHN, "LOITER CHANGE"},
    {FlightMode::MSPRCOVERRIDE, "MSP RC OVERRIDE"}
    
};



#endif
