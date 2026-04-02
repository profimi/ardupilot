#include "rf_power.h"
#include "Plane.h"
#include <string.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/Util.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_VideoTX/AP_VideoTX.h>

extern const AP_HAL::HAL& hal;  // Required for snprintf()
// extern Plane plane;  // Required for the optional crash check

/// @brief Whether the battery state is critical
/// @return Whether the vehicle battery state is critical
static bool is_battery_critical() {
    AP_BattMonitor &battery = AP::battery();
    return battery.has_failsafed() && battery.get_highest_failsafe_priority() >= (uint8_t)AP_BattMonitor::Failsafe::Critical;
}

#if RF_POWERSWITCH_ENABLED
const AP_Param::GroupInfo RF_PowerSwitch::var_info[] = {
    // @Param: OFF_TIME1
    // @DisplayName: RF (TX & VTX) power off mode 1 duration, min
    // @Description: RF (TX & VTX) power off duration in mode 1, minutes; 0 - disabled (always on)
    // @Range 0 255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("OFF_TIME1", 0, RF_PowerSwitch, off_time1, RF_POWER_OFF_TIME1),

    // @Param: OFF_TIME2
    // @DisplayName: RF (TX & VTX) power off mode 2 duration
    // @Description: RF (TX & VTX) power off duration in mode 2, minutes; 0 - disabled (always on)
    // @Range 0 255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("OFF_TIME2", 1, RF_PowerSwitch, off_time2, RF_POWER_OFF_TIME2),

    // @Param: DEV_TIME2
    // @DisplayName: RF (TX & VTX) power off deviation (~3 STD) for mode 2
    // @Description: RF (TX & VTX) power off deviation (range bound) for mode 2, minutes < off_time2; 0 - disabled (exact time)
    // @Range 0 OFF_TIME2
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DEV_TIME2", 2, RF_PowerSwitch, dev_time2, RF_POWER_DEV_TIME2),

    // @Param: ON_TIME
    // @DisplayName: RF (TX & VTX) power on minimal duration, sec
    // @Description: RF (TX & VTX) power on minimal duration, seconds (0, 1, 5-10 are the typical values);
    // 0 - disable the limitation (permanent off until the mode switching), 1 - infinity (permanent on until the mode switching),
    // 5 is a good value for experienced pilots to reduce RF emission still controlling the flight;
    // Note: it takes around 2 sec to power on and enable VTX
    // @Range 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ON_TIME", 3, RF_PowerSwitch, on_time, RF_POWER_ON_TIME),

    // @Param: SAFE_ALT
    // @DisplayName: Safe altitude for RF (TX & VTX) powering off
    // @Description: Minimal safe altitude relative to the home point for RF (TX & VTX) powering off, m
    // @Range -32768 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_ALT", 4, RF_PowerSwitch, safe_alt, RF_POWER_SAFE_ALT),

    // @Param: SAFE_VSPEED
    // @DisplayName: Safe vertical speed for RF (TX & VTX) powering off
    // @Description: Maximal safe vertical speed for RF (TX & VTX) powering off, m/s; 0 - disabled
    // @Range 0 255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_VSPEED", 5, RF_PowerSwitch, safe_vspeed, RF_POWER_SAFE_VSPEED),

    // @Param: SAFE_PITCH
    // @DisplayName: Safe pitch angle for RF (TX & VTX) powering off
    // @Description: Maximal absolute pitch angle for RF (TX & VTX) powering off, deg; 0 - disabled
    // @Range 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_PITCH", 6, RF_PowerSwitch, safe_pitch, RF_POWER_SAFE_PITCH),

    // @Param: SAFE_ROLL
    // @DisplayName: Safe roll angle for RF (TX & VTX) powering off
    // @Description: Maximal absolute roll angle for RF (TX & VTX) powering off, deg; 0 - disabled
    // @Range 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_ROLL", 7, RF_PowerSwitch, safe_roll, RF_POWER_SAFE_ROLL),

    // @Param: SAFE_YAW
    // @DisplayName: Safe yaw angle change relative to the one on RF (TX & VTX) powering off
    // @Description: Maximal absolute yaw angle nodule relative to the one on RF (TX & VTX) powering off, deg; 0 - disabled
    // @Range 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_YAW", 8, RF_PowerSwitch, safe_yaw, RF_POWER_SAFE_YAW),

    // @Param: CTL_GPIO
    // @DisplayName: GPIO that de/activates the RF (TX & VTX) power switch
    // @Description: GPIO that de/activates the RF (TX & VTX) power switch for the integration with fiber optic controlled RF powering; -1 = 255 - disabled
    // @Range 0 255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CTL_GPIO", 9, RF_PowerSwitch, ctl_gpio, RF_POWER_CTL_GPIO),

    // @Param: OFF_NOFS
    // @DisplayName: Disable failsafe triggering on powering off RF communication
    // @Description: Disable failsafe triggering when RF communication is powered off to retain current flight mode
    // @Values: 0:Disable, 1:Enable
    // @User: Standard
    AP_GROUPINFO("OFF_NOFS", 10, RF_PowerSwitch, off_nofs, RF_POWER_OFF_NOFS),

    // @Param: OFF_DELAY
    // @DisplayName: RF (TX & VTX) power off delay, dms
    // @Description: RF (TX & VTX) power off delay, dozen miliseconds (30-80 is recommended to report scheduled shutdown time to GCS).
    // Typically, it should be lower than OFF_TIMEx * 1000.
    // @Range 0 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFF_DELAY", 11, RF_PowerSwitch, off_delay, RF_POWER_OFF_DELAY),

    // @Param: OFF_TDIV
    // @DisplayName: RF (TX & VTX) power off modes duration divider (ratio)
    // @Description: RF (TX & VTX) power off modes duration divider (ratio) for the faster testing: 1 - min, 6 - dozen sec, 60 - sec
    // @Range 1 255
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFF_TDIV", 12, RF_PowerSwitch, off_tdiv, RF_POWER_OFF_TIMEDIV),

    AP_GROUPEND
};

RF_PowerSwitch::RF_PowerSwitch()
: alt{0}, vspeed{0}, pitch{0}, roll{0}, dyaw{0}, off_time{0}, switch_time{0}, power{Power::ON}
, text{0}, failsafe(nullptr), is_crashed(nullptr)
{}

bool RF_PowerSwitch::is_safe(bool partial) const
{
    return alt >= safe_alt && (safe_vspeed == 0 || abs(vspeed) <= (uint8_t)safe_vspeed) && (safe_pitch == 0 || abs(pitch) <= (uint8_t)safe_pitch)
         && (safe_roll == 0 || abs(roll) <= (uint8_t)safe_roll) && (partial || safe_yaw == 0 || abs(dyaw) < (uint8_t)safe_yaw) && !is_critical();
}

bool RF_PowerSwitch::is_critical() const
{
    return (is_crashed && *is_crashed) || is_battery_critical();
}

void RF_PowerSwitch::init(AP_Enum<Failsafe>* rf_failsafe, bool* crashed)
{
    // if(off_nofs)  // Note: Ignore this flag here to enabling dynamic control by this flag
    failsafe = rf_failsafe;
    is_crashed = crashed;
    AP::relay()->set(AP_Relay_Params::FUNCTION::RF_POWER, true);
    // Validate input parameter for errros
    if(!off_tdiv) {
        off_tdiv.set_and_save(1);
        strncpy(text, "Invalid input off_tdiv=0 -> 1 corrected", sizeof(text)-1);
    }
}

bool RF_PowerSwitch::process(RC_Channel::AuxSwitchPos spos)
{
    static RC_Channel::AuxSwitchPos last_spos = RC_Channel::AuxSwitchPos::LOW;
    static uint32_t last_tms = 0;  // Last time in ms of the successful RF power switching

    // // Find the GPIO ID for Servo 10 (index 9)
    // int8_t gpio_pin;
    // if (!SRV_Channels::get_gpio(9, gpio_pin)) {
    //     return; // Servo 10 is not capable of GPIO or not configured as -1
    // }
    // Returns 1 for HIGH (3.3V/5V), 0 for LOW (0V)
    if(ctl_gpio != -1 && hal.gpio->read(ctl_gpio)) {
        strncpy(text, "No RFPW switching: HIGH ctl_gpio", sizeof(text)-1);
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();  // Time since boot in milliseconds
    // Igore the same mode until swithing to another one
    // Intentionally retain power on for rf_on_time == 1 if the RF tumbler has not been switched to another position
    if(last_spos == spos && is_on_infinite()) {  // permanent, continuous, standing
        last_tms = now_ms;
        strncpy(text, "No RFPW switching: infinite on_time=1", sizeof(text)-1);
        return false;
        // *text = 0;
        // return true;
    }

    if(spos == RC_Channel::AuxSwitchPos::LOW) {
        last_tms = now_ms;
        // const bool switched = last_spos != spos;  // Note: This might be also the return movement of the switch when sticky mode is used
        last_spos = spos;
        // plane.failsafe.rc_failsafe_active = true;
        *text = 0;
        return false;  // Note: the switching happens in periodic()
    }

    // Ensure the RF power was available for at least several seconds if rf_on_time >= 1
    // Power off time in minutes, 0 - disable (always on)
    if(spos == RC_Channel::AuxSwitchPos::HIGH) {
#ifdef RAND_SEEDED
        srand(now_ms);  // Introduce random seed for the power switch
#endif
        off_time = get_random_uniform(uint8_t(off_time2) * 60, uint8_t(dev_time2) * 60);
        // get_random_normal(off_time2 * 60, dev_time2 * 60 / 3);  // Note: STD ~<= bound / 3 
    } else off_time = uint8_t(off_time1) * 60;

    // Note: on_time == 1 is considered above by is_on_infinite()
    if(last_spos == spos && now_ms - last_tms < MAX(off_time / off_tdiv, on_time) * 1000) {
        *text = 0;
        return false;
    }

    // Ensure it is safe to power off RF
    update_safety_vals();
    if(!is_safe(true)) {
        hal.util->snprintf(text, sizeof(text), "No RFPW off: unsafe (pitch: %d, roll: %d, dyaw: %d, alt: %d, vs: %d, crbat: %u, crash: %u)"
            , pitch, roll, dyaw, alt, vspeed, is_battery_critical(), is_crashed && *is_crashed);
        return false;
    }

    last_spos = spos;
    last_tms = now_ms;

    if(!off_time) {
        strncpy(text, "No RFPW off: 0 means omitted", sizeof(text)-1);
        return false;  // There is no much sense to notify about omission of power of for 0 min
    }
    hal.util->snprintf(text, sizeof(text), "RFPW is off for %u min %u sec", off_time / (off_tdiv*60), (off_time / off_tdiv) % 60);
    power = Power::DEACTIVATING;
    switch_time = now_ms;
    return true;
}

void RF_PowerSwitch::update_safety_vals()
{
    const AP_AHRS& ahrs = AP::ahrs();

    float alt_home = 0;
    ahrs.get_relative_position_D_home(alt_home);  // get_velocity_D; getCorrectedDeltaVelocityNED; get_velocity_NED
    // AP::ahrs().get_location(loc)
    // alt_home = loc.alt * 0.01f;  // Convert cm to meters
    alt = -roundf(alt_home);  // constrain_float(alt_home, INT16_MIN, INT16_MAX)
    
    float vel = 0;
    if(!ahrs.get_velocity_D(vel)) {
        // This is an estimated value, which might be inaccurate or invalid
    }
    vspeed = constrain_int16(-roundf(vel), INT8_MIN, INT8_MAX);

    pitch = constrain_int16(ahrs.get_pitch_deg(), INT8_MIN, INT8_MAX);  // Negative for descend
    roll = constrain_int16(ahrs.get_roll_deg(), INT8_MIN, INT8_MAX);  // Negative for the left roll (counter clockwise)
}

void RF_PowerSwitch::periodic()
{
    static Failsafe fsval_orig;
    static int16_t yaw_off;  // Yaw value on RF powering off

    // Check crash status
    const uint32_t now_ms = AP_HAL::millis();  // Time since boot in milliseconds
    switch(power) {
    case Power::DEACTIVATING:
        {
            // const AP_AHRS& ahrs = AP::ahrs();
            // ATTENTION: Intentionally allow control with unhealthy AHRS
            // if(!ahrs.healthy()) {
            //     // strncpy(text, "No RFPW off: unsafe deactivation (unhealthy AHRS)", sizeof(text)-1);
            //     GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "%s", "No RFPW off: unsafe deactivation (unhealthy AHRS)");
            //     break;
            // }
            // Introduce a small delay on powering off (50ms) to secure transfer of the powering off notification with the expected timing to the GCS
            if(now_ms < switch_time + uint8_t(off_delay)*10)  // Note: 30ms  is not sufficient for GSC reporting
                break;
            power = Power::OFF;
            switch_time = now_ms;
            AP::relay()->set(AP_Relay_Params::FUNCTION::RF_POWER, false);
            yaw_off = AP::ahrs().get_yaw_deg();
        }

        // The state can be checked externally by AP::relay()->enabled(AP_Relay_Params::FUNCTION::RF_POWER);  // However, that call is slow
        // Disable the failsafe state if necessary
        if(off_nofs && failsafe) {
            fsval_orig = *failsafe;
            failsafe->set(Failsafe::EnabledNoFS);  // That is throttle_fs_enabled
            // // FS_GCS_ENABLE = 0   // Disable GCS failsafe entirely
            // // // Note: FS_LONG_TIMEOUT = 0 // Long failsafe is activated at once
            // // plane.g.throttle_failsafe.set_and_save(0);
            //
            // // Handles physicall loss of RF connectifity
            // plane.g.throttle_fs_enabled = Failsafe::EnabledNoFS;
            //
            // // Prevent GCS failsafe from triggering by updating heartbeat timestamp (prevents timeout)
            // plane.failsafe.last_heartbeat_ms = AP_HAL::millis();
            // // // Directly disable GCS failsafe flag
            // // plane.failsafe.gcs = false;
            // // Prevent RC failsafe if using physical RC link
            // plane.failsafe.last_valid_rc_ms = AP_HAL::millis();
        }
        // Seamlessly switching to the power off
        FALLTHROUGH;
    case Power::OFF: {
        // const AP_AHRS& ahrs = AP::ahrs();
        // ATTENTION: Intentionally allow control with unhealthy AHRS
        // if(ahrs.healthy()) {
            // Update the flight safety values (altitude and pitching)
            {
                update_safety_vals();
                dyaw = wrap_180(AP::ahrs().get_yaw_deg() - yaw_off);  // -180 .. 180
            }

            if(is_safe(false)) {
                if(now_ms < switch_time + off_time * 1000 / off_tdiv)
                    break;
            } else {
                // hal.util->snprintf(text, sizeof(text), "Emergency RFPW on (alt: %d, vspeed: %d, pitch: %d, dyaw: %d, crbat: %u, crash: %u)",
                //     alt, vspeed, pitch, dyaw, is_battery_critical(), is_crashed && *is_crashed);
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Emergency RFPW on (alt: %d, vspeed: %d, pitch: %d, roll: %d, dyaw: %d, crbat: %u, crash: %u)",
                    alt, vspeed, pitch, roll, dyaw, is_battery_critical(), is_crashed && *is_crashed);
            }
        // } else {
        //     // strncpy(text, "Emergency RFPW on: unhealthy AHRS", sizeof(text)-1);
        //     GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "%s", "Emergency RFPW on: unhealthy AHRS");
        // }
        // Seamlessly switching to the power activation
        FALLTHROUGH;
    }
    case Power::ACTIVATING:
        AP::relay()->set(AP_Relay_Params::FUNCTION::RF_POWER, true);
        power = Power::ON;
        switch_time = now_ms;
        // Recover the failsafe mode if it was diabled on power deactivation
        if(off_nofs && failsafe)
            failsafe->set(fsval_orig);
        break;
    default:
        if(is_critical()) {
            AP_Notify::flags.vehicle_lost = true;  // Continuous beep pattern
            if(is_battery_critical() && !AP::vtx().get_pitmode()) {
                // Set VTX to PIT mode to save battery
                AP_VideoTX &vtx = AP::vtx();
                // To turn ON pit mode
                if(!vtx.get_enabled())
                    vtx.set_enabled(true); // Ensure VTX control is enabled
                // vtx.change_power(0);
                vtx.set_options(vtx.get_options() & (uint8_t)AP_VideoTX::VideoOptions::VTX_PITMODE);
            }
        }
        break;
    }
}
#endif  // RF_POWERSWITCH_ENABLED
