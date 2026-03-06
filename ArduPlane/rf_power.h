#pragma once

// #include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>

#if AP_RELAY_ENABLED

#ifndef RF_POWERSWITCH_ENABLED
#define RF_POWERSWITCH_ENABLED 1
#endif

class RF_PowerSwitch {
public:
    enum class Power: uint8_t {
        OFF,
        ON,
        ACTIVATING,
        DEACTIVATING,
    };

    // CAUTION: This class is refactored from Parameters::ThrFailsafe and used heavily in Parameters
    enum class Failsafe: uint8_t {
        Disabled    = 0,
        Enabled     = 1,
        EnabledNoFS = 2
    };

    static const struct AP_Param::GroupInfo var_info[];

    RF_PowerSwitch();
    // CLASS_NO_COPY(RF_PowerSwitch);
    
    /// @brief Infinite on
    /// @return Infinite on is enabled until the state change
    bool is_on_infinite() const  { return on_time == 1; }

    /// @brief Whether the powering off is safe
    /// @return Whether the powering off is safe considering the altitude, vspeed, pitch
    bool is_safe() const;

    /// @brief Whether the vehicle state is critical
    /// @return Whether the vehicle state is critical (unsafe); buzzer is enabled and the VTX power might be reduced
    bool is_critical() const;

    /// @brief Failsafe is disabled in powered off RF switch
    /// @return Failsafe is disabled when RF communication is powered off to retain the flight mode
    bool is_nofailsafe() const  { return off_nofs; }

    /// @brief Init the RF poweroff swich, powering it on start and providing the failsafe state control on switching
    /// @param[in, out] rf_failsafe  - RF failsafe state
    /// @param[in] crashed  - whether the vehicle is crashed
    void init(AP_Enum<Failsafe>* rf_failsafe, bool* crashed);

    /// @brief Process RC signal to switch the RF power
    /// @param spos  - RC switch position
    /// @return whether the power was switched
    bool process(RC_Channel::AuxSwitchPos spos);

    /// @brief A function called by the main thread periodically to estimate safety and manage the power off duration
    void periodic();

    /// @brief Processing result message
    /// @return textual description of the last process() call result
    const char *msg() const  { return text; }
private:
    // Parameters
    AP_Int8 off_time1;  // min
    AP_Int8 off_time2;  // min
    AP_Int8 dev_time2;  // min, < off_time2
    AP_Int8 on_time;  // sec (0: disable, 1: infinity, 2..255 sec)
    AP_Int16 safe_alt;  // m relative to the home point or absolute (relying on the barometer)
    AP_Int8 safe_vspeed;  // m/s
    AP_Int8 safe_pitch;  // deg
    AP_Int8 ctl_gpio;  // Control GPIO that de/activates the power switch
    AP_Int8 off_nofs;  // Disable failsafe triggering on powering off RF communication

    int16_t alt;  // Current altitude
    int8_t vspeed;  // Current vspeed
    int8_t pitch;  // Current pitch angle, deg, negative ground inclination (down is negative)
    uint16_t off_time;  // Scheduling power off time, sec
    uint32_t switch_time;  // Power switching time, ms
    Power power;  // Whether in the power state, when safety checks should be activated
    char text[0xFF];
    AP_Enum<Failsafe>* failsafe;  // Vehicle hardware RF failsafe enabling flag
    const bool* is_crashed;  // Whether the vehicle is crashed

    // Note: 50 ms is insufficent at all (<20% success rate) if the telemetry transfer has not been forced from this endpoint
    // 200 ms is also not always sufficient
    static constexpr uint16_t  POWEROFF_DELAY = 300;  // ms; poweroff delay (latency) to ensure the notification is passed to the GS, including the power off duration
};

#endif  // AP_RELAY_ENABLED