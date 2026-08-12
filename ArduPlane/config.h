#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_APM_HARDWARE
#error CONFIG_APM_HARDWARE option is deprecated! use CONFIG_HAL_BOARD instead.
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


#ifndef FLAP_1_PERCENT
 # define FLAP_1_PERCENT 0
#endif
#ifndef FLAP_1_SPEED
 # define FLAP_1_SPEED 0
#endif
#ifndef FLAP_2_PERCENT
 # define FLAP_2_PERCENT 0
#endif
#ifndef FLAP_2_SPEED
 # define FLAP_2_SPEED 0
#endif
//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
// FLIGHT_MODE_CHANNEL
//
#ifndef FLIGHT_MODE_CHANNEL
 # define FLIGHT_MODE_CHANNEL    8
#endif
#if (FLIGHT_MODE_CHANNEL != 5) && (FLIGHT_MODE_CHANNEL != 6) && (FLIGHT_MODE_CHANNEL != 7) && (FLIGHT_MODE_CHANNEL != 8)
 # error XXX
 # error XXX You must set FLIGHT_MODE_CHANNEL to 5, 6, 7 or 8
 # error XXX
#endif

#if !defined(FLIGHT_MODE_1)
 # define FLIGHT_MODE_1                  Mode::Number::RTL
#endif
#if !defined(FLIGHT_MODE_2)
 # define FLIGHT_MODE_2                  Mode::Number::RTL
#endif
#if !defined(FLIGHT_MODE_3)
 # define FLIGHT_MODE_3                  Mode::Number::FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_4)
 # define FLIGHT_MODE_4                  Mode::Number::FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_5)
 # define FLIGHT_MODE_5                  Mode::Number::MANUAL
#endif
#if !defined(FLIGHT_MODE_6)
 # define FLIGHT_MODE_6                  Mode::Number::MANUAL
#endif


//////////////////////////////////////////////////////////////////////////////
// AUTO_TRIM
//
#ifndef AUTO_TRIM
 # define AUTO_TRIM                              DISABLED
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY
//
#ifndef GROUND_START_DELAY
 # define GROUND_START_DELAY             0
#endif

#ifndef DSPOILR_RUD_RATE_DEFAULT
 #define DSPOILR_RUD_RATE_DEFAULT 100
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE
//
#ifndef AIRSPEED_CRUISE
 # define AIRSPEED_CRUISE                12 // 12 m/s
#endif



//////////////////////////////////////////////////////////////////////////////
// MIN_GROUNDSPEED
//
#ifndef MIN_GROUNDSPEED
 # define MIN_GROUNDSPEED                   0 // m/s (0 disables)
#endif


//////////////////////////////////////////////////////////////////////////////
// FLY_BY_WIRE_B airspeed control
//
#ifndef AIRSPEED_FBW_MIN
 # define AIRSPEED_FBW_MIN               9
#endif
#ifndef AIRSPEED_FBW_MAX
 # define AIRSPEED_FBW_MAX               22
#endif

#ifndef CRUISE_ALT_FLOOR
 # define CRUISE_ALT_FLOOR 0
#endif


//////////////////////////////////////////////////////////////////////////////
// Servo Mapping
//
#ifndef THROTTLE_MIN
 # define THROTTLE_MIN                   0 // percent
#endif
#ifdef THROTTLE_CRUISE
#error THROTTLE_CRUISE was renamed to AP_PLANE_TRIM_THROTTLE_DEFAULT
#endif
#ifndef AP_PLANE_TRIM_THROTTLE_DEFAULT
 # define AP_PLANE_TRIM_THROTTLE_DEFAULT                45
#endif
#ifndef THROTTLE_MAX
 # define THROTTLE_MAX                   100
#endif

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
#ifndef ROLL_LIMIT_DEG
 # define ROLL_LIMIT_DEG                         45
#endif
#ifndef PITCH_MAX
 # define PITCH_MAX                              20
#endif
#ifndef PITCH_MIN
 # define PITCH_MIN                              -25
#endif

#ifndef RUDDER_MIX
 # define RUDDER_MIX           0.5f
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Logging control
//

#define DEFAULT_LOG_BITMASK   0xffff


//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
#ifndef WP_RADIUS_DEFAULT
 # define WP_RADIUS_DEFAULT              90
#endif

#ifndef LOITER_RADIUS_DEFAULT
 # define LOITER_RADIUS_DEFAULT 60
#endif

#ifndef ALT_HOLD_HOME
 # define ALT_HOLD_HOME 100
#endif

//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

#ifndef SCALING_SPEED
 # define SCALING_SPEED          15.0
#endif

// a digital pin to set high when the geo-fence triggers. Defaults
// to -1, which means don't activate a pin
#ifndef FENCE_TRIGGERED_PIN
 # define FENCE_TRIGGERED_PIN -1
#endif

#ifndef AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
 #define AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED 1
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF Failsafe
#ifndef FS_EKF_THRESHOLD_DEFAULT
 # define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

/////////////////////////////////////////////////////////////////////////////
//  Landing Throttle Control Trigger Threshold
#ifndef THR_CTRL_LAND_THRESH
 #define THR_CTRL_LAND_THRESH 0.7
#endif

/////////////////////////////////////////////////////////////////////////////
// Custom items
#ifndef TAKEOFF_UNSAFE
    #define TAKEOFF_UNSAFE 0
#endif

#ifndef THROTTLE_ALT_MIN
    #define THROTTLE_ALT_MIN 10.0f
#endif

#ifndef PREARM_FMODE_CTL
    #define PREARM_FMODE_CTL 0
#endif

#ifndef RF_POWER_OFF_TIME1
    #define RF_POWER_OFF_TIME1 2
#endif

#ifndef RF_POWER_OFF_TIME2
    #define RF_POWER_OFF_TIME2 12
#endif

#ifndef RF_POWER_DEV_TIME2
    #define RF_POWER_DEV_TIME2 3
#endif

#ifndef RF_POWER_ON_TIME
    #define RF_POWER_ON_TIME 5  // 5 sec is a good value for experienced pilots to reduce RF emission still controlling the flight; 5-10 sec (0: disable, 1: infinity, 2..255 sec); Note: it takes around 2 sec to power on and enable VTX;
#endif

#ifndef RF_POWER_SAFE_ALT
    #define RF_POWER_SAFE_ALT 12
#endif

#ifndef RF_POWER_SAFE_VSPEED
    #define RF_POWER_SAFE_VSPEED 5
#endif

#ifndef RF_POWER_SAFE_PITCH
    #define RF_POWER_SAFE_PITCH 20
#endif

#ifndef RF_POWER_SAFE_ROLL
    #define RF_POWER_SAFE_ROLL 20
#endif

#ifndef RF_POWER_SAFE_YAW
    #define RF_POWER_SAFE_YAW 20
#endif

#ifndef RF_POWER_CTL_GPIO
    #define RF_POWER_CTL_GPIO 59  // Servo10 pin
#endif

#ifndef RF_POWER_OFF_NOFS
    #define RF_POWER_OFF_NOFS 0  // false
#endif

#ifndef RF_POWER_OFF_DELAY
    #define RF_POWER_OFF_DELAY 50  // 30*10 ms is also not always sufficient
#endif

#ifndef RF_POWER_OFF_TIMEDIV
    #define RF_POWER_OFF_TIMEDIV 1  // 1 - min, 6 - dozen sec, 60 - sec
#endif

// #ifndef DIRLOCK_RCIN
//     #define DIRLOCK_RCIN 16
// #endif

#ifndef DEMO_GROUND_THR
    #define DEMO_GROUND_THR 0  // Allow ground throttle in failsafe modes for the demo
#endif

#ifndef DEMO_GROUND_THR
    #define DEMO_GROUND_THR 0  // Allow ground throttle in failsafe modes for the demo
#endif

#ifndef FBWT_AIRSPD_MIN
    #define FBWT_AIRSPD_MIN  15
#endif

#ifndef FBWT_ALT_MIN
    #define FBWT_ALT_MIN  30
#endif

#ifndef FBWT_PITCH_MIN
    #define FBWT_PITCH_MIN  -50  // -40..-60, 30..40
#endif

#ifndef CTL_EXPOCRV
    #define CTL_EXPOCRV  0.3f
#endif
