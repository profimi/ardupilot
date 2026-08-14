#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>
#include "quadplane.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>
#include "config.h"
#include "pullup.h"
#include "systemid.h"

#ifndef AP_QUICKTUNE_ENABLED
#define AP_QUICKTUNE_ENABLED HAL_QUADPLANE_ENABLED
#endif

#ifndef MODE_AUTOLAND_ENABLED
#define MODE_AUTOLAND_ENABLED 1
#endif

#include <AP_Quicktune/AP_Quicktune.h>

class AC_PosControl;
class AC_AttitudeControl_Multi;
class AC_Loiter;
class Mode
{
public:

    /* Do not allow copies */
    CLASS_NO_COPY(Mode);

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
#if HAL_QUADPLANE_ENABLED
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
#if QAUTOTUNE_ENABLED
        QAUTOTUNE     = 22,
#endif
        QACRO         = 23,
#endif
        THERMAL       = 24,
#if HAL_QUADPLANE_ENABLED
        LOITER_ALT_QLAND = 25,
#endif
#if MODE_AUTOLAND_ENABLED
        AUTOLAND      = 26,
#endif

        // Custom modes
        FLY_BY_WIRE_C = 28,  // ATTENTION: FLTMODE3/4 should be synchronously set to this value to support Flight mode switching from RC
        FLY_BY_WIRE_T = 29,  // ATTENTION: FLTMODEn or respective AUX function's RC switch should be synchronously set to this value to support Flight mode switching from RC
        // Mode number 30 reserved for "offboard" for external/lua control.
        FLY_BY_WIRE_L = 31,  // ATTENTION: FLTMODEn or respective AUX function's RC switch should be synchronously set to this value to support Flight mode switching from RC
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // run controllers specific to this mode
    virtual void run();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    bool pre_arm_checks(size_t buflen, char *buffer) const;

    // Reset rate and steering and TECS controllers
    void reset_controllers();

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }

    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }
    
    // true if the mode supports autotuning (via switch for modes other
    // that AUTOTUNE itself
    virtual bool mode_allows_autotuning() const { return false; }

    // method for mode specific target altitude profiles
    virtual void update_target_altitude();

    // handle a guided target request from GCS
    virtual bool handle_guided_request(Location target_loc) { return false; }

    // true if is landing 
    virtual bool is_landing() const { return false; }

    // true if is taking 
    virtual bool is_taking_off() const;

    // true if throttle min/max limits should be applied
    virtual bool use_throttle_limits() const;

    // true if voltage correction should be applied to throttle
    virtual bool use_battery_compensation() const;
 
#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    virtual bool allows_autoland_direction_capture() const { return false; }
#endif

#if AP_QUICKTUNE_ENABLED
    // does this mode support VTOL quicktune?
    virtual bool supports_quicktune() const { return false; }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    virtual bool supports_vtol_systemid() const { return false; }

    // does this mode support plane or quadplane fixed wing systemid?
    virtual bool supports_fw_systemid() const { return false; }

    // Return true if fixed wing system ID should be allowed
    bool allow_fw_systemid() const;
#endif

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // mode specific pre-arm checks
    virtual bool _pre_arm_checks(size_t buflen, char *buffer) const;

    // Helper to output to both k_rudder and k_steering servo functions
    void output_rudder_and_steering(float val);

    // Output pilot throttle, this is used in stabilized modes without auto throttle control
    void output_pilot_throttle();

    // makes the initialiser list in the constructor manageable
    uint8_t unused_integer;

#if HAL_QUADPLANE_ENABLED
    // References for convenience, used by QModes
    AC_PosControl*& pos_control;
    AC_AttitudeControl_Multi*& attitude_control;
    AC_Loiter*& loiter_nav;
    QuadPlane& quadplane;
    QuadPlane::PosControlState &poscontrol;
#endif
    AP_AHRS& ahrs;
};


class ModeAcro : public Mode
{
friend class ModeQAcro;
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "Acro"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    void stabilize();

    void stabilize_quaternion();

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

protected:

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
        Quaternion q;
        bool roll_active_last;
        bool pitch_active_last;
        bool yaw_active_last;
    } acro_state;

    bool _enter() override;
};

//  Plane follows a mission
class ModeAuto : public Mode
{
public:
    friend class Plane;

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "Auto"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override;

    bool does_auto_throttle() const override;
    
    bool mode_allows_autotuning() const override { return true; }

    bool is_landing() const override;

    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    bool verify_altitude_wait(const AP_Mission::Mission_Command& cmd);

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    bool in_pullup() const { return pullup.in_pullup(); }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
    void _exit() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override;

private:

    // Delay the next navigation command
    struct {
        uint32_t time_max_ms;
        uint32_t time_start_ms;
    } nav_delay;

    // wiggle state and timer for NAV_ALTITUDE_WAIT
    void wiggle_servos();
    struct {
        uint8_t stage;
        uint32_t last_ms;
    } wiggle;

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    GliderPullup pullup;
#endif // AP_PLANE_GLIDER_PULLUP_ENABLED
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "Autotune"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
    
protected:

    bool _enter() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "Guided"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // handle a guided airspeed command, typically from companion computer
    bool handle_change_airspeed(const float airspeed, const float acceleration);
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

    void set_radius_and_direction(const float radius, const bool direction_is_ccw);

    void update_target_altitude() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return true; }
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif

private:
    float active_radius_m;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "Circle"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "Loiter"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    bool allows_terrain_disable() const override { return true; }

    void update_target_altitude() override;
    
    bool mode_allows_autotuning() const override { return true; }

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
};

#if HAL_QUADPLANE_ENABLED
class ModeLoiterAltQLand : public ModeLoiter
{
public:

    Number mode_number() const override { return Number::LOITER_ALT_QLAND; }
    const char *name() const override { return "Loiter to QLand"; }
    const char *name4() const override { return "L2QL"; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:
    bool _enter() override;

    void navigate() override;

private:
    void switch_qland();

};
#endif // HAL_QUADPLANE_ENABLED

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "Manual"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    // true if throttle min/max limits should be applied
    bool use_throttle_limits() const override;

    // true if voltage correction should be applied to throttle
    bool use_battery_compensation() const override { return false; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    // Switch to QRTL if enabled and within radius
    bool switch_QRTL();
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "Stabilize"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

private:
    void stabilize_stick_mixing_direct();

};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "Training"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "Initialising"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FBWA"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FBWB"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }
    
    bool mode_allows_autotuning() const override { return true; }

    void update_target_altitude() override {};

protected:

    bool _enter() override;
};

// Like FBWA but with failsafe FBWC-like flight in case of FBWT_LIMIT_X violation
class ModeFBWC : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_C; }
    const char *name() const override { return "FLY_BY_WIRE_C"; }
    const char *name4() const override { return "FBWC"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // Methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }
    
    bool mode_allows_autotuning() const override { return false; }

    void update_target_altitude() override {};

protected:
    float target_yaw = 0.0f;  // In rad
    bool _enter() override;
};

#ifndef MODE_FBWT_ENABLED
#define MODE_FBWT_ENABLED 1
#endif

class ModeFBWT : public Mode {
public:
    enum class Submode {
        Fbwa,
        Levelup,
        // Headhold,
    };

    // // enum FBWTPhase {  // What how and how is it defined?
    // //     PHASE_TAKEOFF,
    // //     PHASE_CLIMB,
    // //     PHASE_CRUISE,
    // //     PHASE_MANEUVER,
    // //     PHASE_DESCENT,
    // //     PHASE_LANDING
    // // };
    // 
    // // struct EnvelopeLimits {
    // //     float pitch_max;
    // //     float pitch_min;
    // //     float roll_max;
    // // };

    struct EnvelopeState {
        float aoa;              // deg
        float aoa_limit;        // deg
        float aoa_margin;       // normalized (0–1)

        float load_factor;      // G
        float g_limit;          // max allowed
        float g_margin;         // normalized

        float airspeed;         // m/s
        float v_min;            // stall buffer speed
        float v_margin;         // normalized

        float energy_error;     // TECS total energy error
        float energy_margin;    // normalized

        uint8_t limiter_flags;  // bitmask (AoA, G, Energy, Speed)
    };

    // friend class Plane;

    // ModeFBWT(Plane &plane);

    Number mode_number() const override { return Number::FLY_BY_WIRE_T; }
    const char *name() const override { return "FBWT"; }
    const char *name4() const override { return "FBWT"; }

    void update() override;
    void run() override;
    // void navigate() override;

    bool mode_allows_autotuning() const override { return _submode == Submode::Fbwa; }

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return mode_allows_autotuning(); }
#endif

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return _submode != Submode::Levelup; }
#endif

    // Levelup and Headhold are auto-throttle (TECS-driven); Fbwa is manual throttle
    bool does_auto_throttle() const override { return _submode != Submode::Fbwa; }

    // // Headhold only actively steers via the navigation controller when it
    // // actually has a usable position to feed it (see have_position()); if
    // // GPS/position is unavailable this evaluates to false and update()
    // // instead runs a direct compass-only heading-hold fallback.
    // bool does_auto_navigation() const override { return _submode == Submode::Headhold && have_position(); }
    //
    // // Don't let stick-mixing fight the bailout/hold controller
    // bool allows_throttle_nudging() const override { return _submode == Submode::Headhold; }

    Submode get_submode() const { return _submode; }

    // Update_target_altitude() is deliberately a no-op, like FBWB/CRUISE:
    // the Headhold target altitude is latched once (in run_levelup()) and
    // is not ramped every loop from a pilot input.
    void update_target_altitude() override {}

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // headhold(bool &isDirLocked, bool doPitchLock);  // Execute headhold

    bool _enter() override;
    void _exit() override;
private:
    // struct EnvelopeLimits {
    //     float aoa_max;
    //     float n_max;
    //     float energy_min;
    //     float energy_max;
    //     float limiter_floor;
    // };

    Submode _submode = Submode::Fbwa;
    AP_TECS* _tecs = nullptr;

    // Parameters
    // const uint8_t alt_hyst = 10;  // m
    AP_Int8 alt_min;  // 30 - 50 m
    // // AP_Int8 pitch_min;  // -60, -50 deg; pitch_max ~45 deg
    // AP_Int8 pitch_max;
    // AP_Int8 roll_max;  // 25 .. 45
    AP_Float ctl_expocrv;  // 0.3f; 0 means disabled

    // Protection
    float estimate_beta() const;
    void apply_dynamic_stall(float &pitch_cmd, float Nz);
    void apply_stall_protection(float &pitch_cmd, float airspeed, float Nz);
    float compute_adaptive_nz_limit(float airspeed);
    void apply_nz_limit(float &roll_cmd, float airspeed);
    void apply_energy_roll_limit(float &roll_cmd, float airspeed);
    void apply_terrain_protection(float &pitch_cmd);
    void apply_flare_protection(float &pitch_cmd);
    void apply_energy_limiter(float &pitch_cmd);
    void apply_turn_coordination(float roll_cmd);
    float get_envelope_style();
    void apply_envelope_shaping(float &roll_lim, float &pitch_lim);

    bool is_below_alt_min() const;
    void apply_levelup_protection(float &roll_cmd, float &pitch_cmd);
    void apply_final_envelope(float &roll_cmd, float &pitch_cmd);

    float estimate_aoa() const;
    float get_fused_aoa() const;
    void apply_aoa_protection(float &pitch_cmd, float aoa);
    void apply_aoa_rate_damping(float &pitch_cmd);
    void apply_aoa_limiter(float &pitch_cmd);

    float energy_rate() const;
    float energy_factor() const;
    float compute_recovery_throttle() const;

    void apply_envelope_limits(float &pitch, float &roll);
    void update_envelope_state(EnvelopeState &env);
    float get_throttle_assist_gain() const;
    void output_fbwt_throttle_assist();

    void log_envelope(const EnvelopeState &env) const;

    // // AoA limits
    // float alpha_stall = radians(15.0f);
    // float alpha_limit = radians(12.0f);

    // // G-limits
    // float nz_max = 4.0f;
    // float nz_min = -2.0f;

    // // Assist gains
    // float assist_gain = 1.0f;
    // float assist_decay = 0.995f;

    // // State
    // float target_alt = 0;
    // float target_heading = 0;
    // float cur_airspeed = 0;  // Current airspeed

    // // Headhold hold-state, latched by enter_headhold()/run_levelup()
    // int32_t  locked_heading_cd;
    // uint32_t levelup_enter_ms;
    //
    // // Altitude reference used when home was never set (no GPS/position
    // // ever available during this FBWT activation) - see reference_alt_cm().
    // int32_t  no_home_ref_alt_cm;

    // // Flags
    // bool has_gps = false;
    // bool has_velocity = false;
    // bool has_airspeed = false;

    // // --- Core functions ---
    // void update_submode();

    // void run_fbwa();
    // void run_levelup();
    // void run_headhold();  // run_headhold_heading_only
    //
    // void enter_fbwa();
    // void enter_levelup();
    // void enter_headhold();

    // // --- Protection ---
    // float compute_alpha();
    // float stall_factor(float alpha);
    // float g_limit_factor(float nz);
    // float energy_factor();
    // void apply_envelope(float &pitch, float &roll);
    // float get_aoa() const;
    // float get_flight_path_angle() const;

    // bool have_position() const;
    // // int32_t reference_alt_cm() const;
    // // bool speed_available(float &eas) const;
    // // bool speed_too_low(float &margin_ms) const;
    // // bool is_stalling() const;
    // // bool pitch_too_low() const;
    // // bool alt_too_low() const;
    // // bool bailout_condition(bool &stalling) const;

    // // --- Helpers ---
    // FBWTPhase detect_phase() const; 
    // EnvelopeLimits get_phase_limits(FBWTPhase phase) const;
    // float compute_envelope_limiter() const;
    // float get_load_factor() const;
    // float energy_rate() const;
    // float compute_energy_limiter() const;
    // float compute_energy_factor() const;
    // float compute_energy() const;
};

// class ModeFBWT : public Mode
// {
//     enum class Submode {
//         Levelup,  // Bailout; Recover 
//         Headhold,  // Althold
//         Fbwa
//     };
// protected:
//     Submode submode;
//     uint16_t alt_max;  // Max reached altitude module
//     uint16_t airspd_max;  // Max reached airspeed
//     uint16_t airspd_min;  // 1.2f * stall ~= 16 m/s
//     // float yaw_targ;  // In rad
// public:
//     ModeFBWT();
//     Number mode_number() const override { return Number::FLY_BY_WIRE_T; }
//     const char *name() const override { return "FBWT"; }
//     const char *name4() const override { return "FBWT"; }

//     // methods that affect movement of the vehicle in this mode
//     void update() override;

//     void run() override;
    
//     bool mode_allows_autotuning() const override { return true; }

// #if AP_PLANE_SYSTEMID_ENABLED
//     // does this mode support fixed wing systemid?
//     bool supports_fw_systemid() const override { return true; }
// #endif

// #if MODE_AUTOLAND_ENABLED   
//     // true if mode allows landing direction to be set on first takeoff after arm in this mode 
//     bool allows_autoland_direction_capture() const override { return true; }
// #endif

//     // var_info for holding parameter information
//     static const struct AP_Param::GroupInfo var_info[];

//     AP_Int8 alt_min;  // 30, 100
//     AP_Int8 pitch_min;  // -50 deg
//     // AP_Int8 airspd_min;  // 1.2f * stall ~= 16 m/s
//     // AP_Float ground_pitch;
// };

#ifndef MODE_FBWL_ENABLED
#define MODE_FBWL_ENABLED 1
#endif

//!   ModeFBWL - "Fly By Wire Learning / Assisted".
//!
//!   Fbwa submode behaves like ModeFBWA (manual roll/pitch, manual
//!   throttle) but with continuous, dynamic envelope protection blended
//!   in: AoA limiter + rate damping, G-limit, stall-speed protection, and
//!   TECS-integrated energy limiting. Coordinated turn / yaw blending is
//!   entirely reused from stock ArduPilot (this mode never writes to
//!   k_rudder). Levelup is a last-resort automatic recovery, engaged only
//!   if the aircraft leaves the envelope despite those protections; once
//!   recovered it hands control straight back to Fbwa. All of the above
//!   degrades gracefully with no GPS and/or no airspeed sensor fitted.
//!
//!   See ArduPlane/mode_fbwl.cpp for the full implementation and rationale,
//!   including the list of stock ArduPlane parameters reused instead of
//!   new ones.
class ModeFBWL : public Mode
{
public:
    Number mode_number() const override { return Number::FLY_BY_WIRE_L; }
    const char *name() const override { return "FBWL"; }
    const char *name4() const override { return "FBWL"; }

    enum class Submode {
        Fbwa,       // assisted (continuously protected) manual flight
        Levelup   // Bailout; Recover
    };

    void update() override;
    void run() override;

    bool mode_allows_autotuning() const override { return submode == Submode::Fbwa; }
    bool does_auto_throttle() const override { return submode != Submode::Fbwa; }
    bool does_auto_navigation() const override { return false; }

    Submode get_submode() const { return submode; }

#if AP_PLANE_SYSTEMID_ENABLED
    bool supports_fw_systemid() const override { return submode == Submode::Fbwa; }
#endif

#if MODE_AUTOLAND_ENABLED
    bool allows_autoland_direction_capture() const override { return submode == Submode::Fbwa; }
#endif

    void update_target_altitude() override {}

    static const struct AP_Param::GroupInfo var_info[];

protected:
    bool _enter() override;
private:
    // ---- tunable safety-envelope parameters (see mode_fbwt.cpp for @Param docs) ----
    AP_Float pitch_min_deg;   // FBWT_PITCH_MIN, deg, default -60
    AP_Float alt_min_m;       // FBWT_ALT_MIN,   m,   default 30
    AP_Float alt_hyst_m;      // FBWT_ALT_HYST,  m,   default 10
    AP_Float aoa_max_deg;     // FBWT_AOA_MAX,   deg, default 12  (new - no stock equivalent)
    AP_Float load_max;        // FBWT_LOAD_MAX,  G,   default 2.5 (new - no stock equivalent)

    Submode submode;

    uint32_t levelup_enter_ms;
    int32_t  no_home_ref_alt_cm;
    mutable bool warned_no_airspeed;

    // AoA rate-damping state
    mutable float    aoa_prev_deg;
    mutable uint32_t aoa_prev_us;
    float aoa_rate_dps_latest;

    // protection-active flags, used by logging + GCS edge-notices
    bool aoa_limit_active, g_limit_active, stall_speed_limit_active, energy_limit_active;
    bool prev_aoa_limit_active, prev_g_limit_active, prev_stall_speed_limit_active, prev_energy_limit_active;
    uint32_t last_log_ms;

    void enter_fbwa();
    void run_fbwa();
    void enter_levelup();
    void run_levelup();

    void apply_aoa_protection();
    void apply_stall_speed_protection();
    void apply_g_limit_protection();
    void apply_energy_limiting();
    void apply_coordinated_yaw() const;
    void log_envelope();

    bool have_position() const;
    int32_t reference_alt_cm() const;
    float alt_too_low_margin_m() const;
    bool speed_available(float &eas) const;
    bool speed_too_low(float &margin_ms) const;
    float estimate_aoa_deg() const;
    bool is_stalling() const;
    bool pitch_too_low() const;
    bool alt_too_low() const;
    bool aoa_exceeded_hard() const;
    bool g_exceeded_hard() const;
    bool bailout_condition(bool &stalling) const;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "Cruise"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override {};

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "Avoid ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

#if HAL_QUADPLANE_ENABLED
class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QStabilize"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHover"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLoiter : public Mode
{
friend class QuadPlane;
friend class ModeQLand;
friend class Plane;

public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLoiter"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
    uint32_t last_target_loc_set_ms;

#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLand : public Mode
{
public:
    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLand"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override;

    bool allows_throttle_nudging() const override;

    float get_VTOL_return_radius() const;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    enum class SubMode {
        climb,
        RTL,
    } submode;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QAcro"; }
    const char *name4() const override { return "QACO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

#if QAUTOTUNE_ENABLED
class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAutotune"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    void run() override;

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};
#endif  // QAUTOTUNE_ENABLED

#endif  // HAL_QUADPLANE_ENABLED

class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "Takeoff"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 target_alt;
    AP_Int16 level_alt;
    AP_Float ground_pitch;
    AP_Float ctl_supr;
    AP_Float ctl_altr;

protected:
    AP_Int16 target_dist;
    AP_Int8 level_pitch;

    bool takeoff_mode_setup;
    Location start_loc;

    bool _enter() override;

private:

    // flag that we have already called autoenable fences once in MODE TAKEOFF
    bool have_autoenabled_fences;

};
#if MODE_AUTOLAND_ENABLED
class ModeAutoLand: public Mode
{
public:
    ModeAutoLand();

    Number mode_number() const override { return Number::AUTOLAND; }
    const char *name() const override { return "Autoland"; }
    const char *name4() const override { return "ALND"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }
    
    bool is_landing() const override;
    
    void check_takeoff_direction(void);

    // return true when lined up correctly from the LOITER_TO_ALT
    bool landing_lined_up(void);

    // see if we should capture the direction
    void arm_check(void);

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 final_wp_alt;
    AP_Int16 final_wp_dist;
    AP_Int16 landing_dir_off;
    AP_Int8  options;
    AP_Int16 terrain_alt_min;

    // Bitfields of AUTOLAND_OPTIONS
    enum class AutoLandOption {
        AUTOLAND_DIR_ON_ARM     = (1U << 0), // set dir for autoland on arm if compass in use.
    };

    enum class AutoLandStage {
        CLIMB,
        LOITER,
        LANDING
    };

    bool autoland_option_is_set(AutoLandOption option) const {
        return (options & int8_t(option)) != 0;
    }

protected:
    bool _enter() override;
    AP_Mission::Mission_Command cmd_climb;
    AP_Mission::Mission_Command cmd_loiter;
    AP_Mission::Mission_Command cmd_land;
    Location land_start;
    AutoLandStage stage;
    void set_autoland_direction(const float heading);
};
#endif
#if HAL_SOARING_ENABLED

class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "Thermal"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif
