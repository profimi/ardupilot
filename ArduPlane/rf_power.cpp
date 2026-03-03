#include "rf_power.h"
#include <string.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/Util.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_VideoTX/AP_VideoTX.h>
// #include <ArduPlane/Plane.h>
#include "Plane.h"

extern const AP_HAL::HAL& hal;  // Required for snprintf()
extern Plane plane;  // Required for the optional crash check

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
    // @Range 0 off_time2
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
    // @Range -32767 32767
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_ALT", 4, RF_PowerSwitch, safe_alt, RF_POWER_SAFE_ALT),

    // @Param: SAFE_VSPEED
    // @DisplayName: Safe vertical speed for RF (TX & VTX) powering off
    // @Description: Maximal safe vertical speed for RF (TX & VTX) powering off, m/s
    // @Range 0 255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_VSPEED", 5, RF_PowerSwitch, safe_vspeed, RF_POWER_SAFE_VSPEED),

    // @Param: SAFE_PITCH
    // @DisplayName: Safe pitch angle for RF (TX & VTX) powering off
    // @Description: Maximal absolute pitch angle for RF (TX & VTX) powering off, deg
    // @Range 0 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFE_PITCH", 6, RF_PowerSwitch, safe_pitch, RF_POWER_SAFE_PITCH),

    // // @Param: RC_FUNC
    // // @DisplayName: Mocking RC function
    // // @Description: Mocking RC function (either an unused one or arming; Ardupilot includes hundreds of them)
    // // @Range -32767 32767
    // // @Increment: 1
    // // @User: Standard
    // AP_GROUPINFO("RC_FUNC", 7, RF_PowerSwitch, rc_func, RC_Channel::AUX_FUNC::ARMDISARM),

    // @Param: CTL_GPIO
    // @DisplayName: GPIO that de/activates the RF (TX & VTX) power switch
    // @Description: GPIO that de/activates the RF (TX & VTX) power switch for the integration with fiber optic controlled RF powering
    // @Range 0 255
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CTL_GPIO", 8, RF_PowerSwitch, ctl_gpio, RF_POWER_CTL_GPIO),

    // @Param: OFF_NOFS
    // @DisplayName: Disable failsafe triggering on powering off RF communication
    // @Description: Disable failsafe triggering when RF communication is powered off to retain current flight mode
    // @Range 0 1
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("OFF_NOFS", 9, RF_PowerSwitch, off_nofs, RF_POWER_OFF_NOFS),

    AP_GROUPEND
};

RF_PowerSwitch::RF_PowerSwitch()
: alt{0}, vspeed{0}, pitch{0}  // , rc_mock{RC_MOCK_NONE}
, off_time{0}, switch_time{0}, power{Power::ON}, text{0}, failsafe(nullptr)
{
    // // Identify RC arming switch
    // for(uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
    //     RC_Channel *chan = RC_Channels::rc_channel(i);
    //     if (chan == nullptr)
    //         continue;
    //
    //     // Check if this channel is assigned to ARM_DISARM (Option 153)
    //     if (chan->option == (uint16_t)rc_func) {
    //         rc_mock = i;
    //         break; // Found it, stop searching
    //     }
    // }
}

bool RF_PowerSwitch::is_safe() const
{
    return alt >= safe_alt && abs(vspeed) <= uint8_t(safe_vspeed) && abs(pitch) <= uint8_t(safe_pitch) && !is_critical();
}

bool RF_PowerSwitch::is_critical() const
{
    return plane.is_crashed() || is_battery_critical();
}

void RF_PowerSwitch::init(AP_Enum<Failsafe>* rf_failsafe)
{
    // if(off_nofs)  // Note: Ignore this flag here to enabling dynamic control by this flag
    failsafe = rf_failsafe;
    AP::relay()->set(AP_Relay_Params::FUNCTION::RF_POWER, true);
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
        strncpy(text, "No RFPW switching: HIGH ctl_gpio", sizeof(text));
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();  // Time since boot in milliseconds
    // Igore the same mode until swithing to another one
    // Intentionally retain power on for rf_on_time == 1 if the RF tumbler has not been switched to another position
    if(last_spos == spos && is_on_infinite()) {  // permanent, continuous, standing
        last_tms = now_ms;
        strncpy(text, "No RFPW switching: infinite on_time=1", sizeof(text));
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
    off_time = spos != RC_Channel::AuxSwitchPos::HIGH
        ? off_time1 * 60 : get_random_uniform(off_time2 * 60, dev_time2 * 60);
        // get_random_normal(off_time2 * 60, dev_time2 * 60 / 3);  // Note: STD ~<= bound / 3 

    if(last_spos == spos && now_ms - last_tms < (off_time + on_time) * 1000) {
        *text = 0;
        return false;
    }

    // Ensure it is safe to power off RF
    if(!is_safe()) {
        hal.util->snprintf(text, sizeof(text), "No RFPW off: unsafe (pitch: %u, alt: %u)", pitch, alt);
        return false;
    }

    last_spos = spos;
    last_tms = now_ms;

    if(!off_time) {
        strncpy(text, "No RFPW off: 0 min is omitted", sizeof(text));
        return false;  // There is no much sense to notify about omission of power of for 0 min
    }
    hal.util->snprintf(text, sizeof(text), "RFPW is off for %u min %u sec", off_time / 60, off_time % 60);
    power = Power::DEACTIVATING;
    switch_time = now_ms;
    return true;
}

void RF_PowerSwitch::periodic()
{
    static Failsafe failsafe_orig;
    // Check crash status
    const uint32_t now_ms = AP_HAL::millis();  // Time since boot in milliseconds
    switch(power) {
    case Power::DEACTIVATING:
        // Introduce a small delay on powering off (50ms) to secure transfer of the powering off notification with the expected timing to the GCS
        if(now_ms < switch_time + POWEROFF_DELAY)  // Note: 50ms  is not sufficient for GSC reporting
            break;
        power = Power::OFF;
        switch_time = now_ms;
        AP::relay()->set(AP_Relay_Params::FUNCTION::RF_POWER, false);
        // The state can be checked externally by AP::relay()->enabled(AP_Relay_Params::FUNCTION::RF_POWER);  // However, that call is slow
        // Disable the failsafe state if necessary
        if(off_nofs && failsafe) {
            failsafe_orig = *failsafe;
            failsafe->set(Failsafe::EnabledNoFS);  // That is throttle_fs_enabled

            // // FS_GCS_ENABLE = 0   // Disable GCS failsafe entirely
            // // FS_LONG_TIMEOUT = 0 // Disable long failsafe
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
    case Power::OFF:
        // Update the flight safety values (altitude and pitching)
        {
            const AP_AHRS& ahrs = AP::ahrs();

            float alt_home = 0;
            ahrs.get_relative_position_D_home(alt_home);  // get_velocity_D; getCorrectedDeltaVelocityNED; get_velocity_NED
            // AP::ahrs().get_location(loc)
            // alt_home = loc.alt * 0.01f;  // Convert cm to meters
            alt = -roundf(alt_home);
            
            float vel = 0;
            if(!ahrs.get_velocity_D(vel)) {
                // This is an estimated value, which might be inaccurate of invalid
            }
            vspeed = -roundf(vel);

            pitch = ahrs.get_pitch_deg();  // Negative for descend
        }

        if(!is_safe()) {
            hal.util->snprintf(text, sizeof(text), "Emergency RFPW on: alt: %d, vspeed: %d, pitch: %d, crit_bar: %u, crashed: %u",
                alt, vspeed, pitch, is_battery_critical(), plane.is_crashed());
        } else if(now_ms < switch_time + off_time * 1000) {
            // Prefent failsafe mode in the RF POWER_OFF state by automatically sending the arming signal
            // if(rc_mock != RC_MOCK_NONE)
            //     RC_Channels::set_override(rc_mock, RC_Channels::rc_channel(rc_mock)->get_radio_max());
            break;
        }
        // Seamlessly switching to the power activation
        FALLTHROUGH;
    case Power::ACTIVATING:
        AP::relay()->set(AP_Relay_Params::FUNCTION::RF_POWER, true);
        power = Power::ON;
        switch_time = now_ms;
        // Recover the failsafe mode if it was diabled on power deactivation
        if(off_nofs && failsafe)
            failsafe->set(failsafe_orig);
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
