uint32 NONE = 0

uint32 ANGLE = 1
uint32 HORIZON = 2
uint32 NAV_POSHOLD = 3
uint32 NAV_CRUISE = 4
uint32 NAV_RTH = 5
uint32 NAV_WP = 6
uint32 MANUAL = 7


uint32 NAV_ALTHOLD = 1
uint32 TURN_ASSIST = 2
uint32 AIR_MODE = 4
uint32 SURFACE = 8
uint32 HEADING_HOLD = 16
uint32 HEADFREE = 32
uint32 HEADADJ = 64


uint32 ARM = 1
uint32 CAMSTAB = 2
uint32 BEEPER = 4
uint32 LEDLOW = 8
uint32 OSD_SW = 16
uint32 TELEMETRY = 32
uint32 BLACKBOX = 64
uint32 FAILSAFE = 128
uint32 HOME_RESET = 256
uint32 GCS_NAV = 512
uint32 FLAPERON = 1024
uint32 NAV_LAUNCH = 2048
uint32 SERVO_AUTOTRIM = 4096
uint32 AUTOTUNE = 8192



Header header
uint32 primary_mode
uint32 secondary_mode
uint32 modifier
