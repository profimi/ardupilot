#pragma once

// #include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <RC_Channel/RC_Channel.h>

#if AP_RELAY_ENABLED

#ifndef RF_POWERSWITCH_ENABLED
#define RF_POWERSWITCH_ENABLED 1
#endif

enum class Power: uint8_t {
    OFF,
    ON,
    ACTIVATING,
    DEACTIVATING,
};

class RF_PowerSwitch {
    // Parameters
    AP_Int8 off_time1;  // min
    AP_Int8 off_time2;  // min
    AP_Int8 dev_time2;  // min, < off_time2
    AP_Int8 on_time;  // sec (0: disable, 1: infinity, 2..255 sec)
    AP_Int16 safe_alt;  // m
    AP_Int8 safe_vspeed;  // m/s
    AP_Int8 safe_pitch;  // deg

    int16_t alt;  // Current altitude
    int8_t vspeed;  // Current vspeed
    int8_t pitch;  // Current pitch angle, deg, negative ground inclination (down is negative)
    uint8_t rc_arm;  // Arming RC channel
    uint16_t off_time;  // Scheduling power off time, sec
    uint32_t switch_time;  // Power switching time, ms
    Power power;  // Whether in the power state, when safety checks should be activated
    char text[0xFF];

    static constexpr decltype(RF_PowerSwitch::rc_arm)  RC_ARM_NONE = -1;
public:
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

    /// @brief Process RC signal to switch the RF power
    /// @param spos  - RC switch position
    /// @return whether the power was switched
    bool process(RC_Channel::AuxSwitchPos spos);

    /// @brief A function called by the main thread periodically to estimate safety and manage the power off duration
    void periodic();

    /// @brief Processing result message
    /// @return textual description of the last process() call result
    const char *msg() const  { return text; }
};

#endif  // AP_RELAY_ENABLED