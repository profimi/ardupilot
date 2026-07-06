#include "mode.h"
#include "Plane.h"

const struct AP_Param::GroupInfo ModeFBWT::var_info[] = {
    // @Param: _AIRSPD_MIN
    // @DisplayName: FBWT minimal airspeed (recommended: 1.2f * stall_speed)
    // @Description: Minimum airspeed for the regular Submode::Fbwa, otherwise Submode::Headhold is activated 
    // @Units: m/s
    // @Range: 10 40
    // @Increment: 1
    AP_GROUPINFO("_AIRSPD_MIN", 1, ModeFBWT, airspd_min, FBWT_AIRSPD_MIN),

    // @Param: _AIRSPD_MIN
    // @DisplayName: FBWT minimal altitude
    // @Description: Minimum airspeed for the regular Submode::Fbwa, otherwise Submode::Headhold is activated. 
    // @Units: m
    // @Range: -128 127
    // @Increment: 1
    AP_GROUPINFO("_ALT_MIN", 2, ModeFBWT, alt_min, FBWT_ALT_MIN),

    // @Param: _AIRSPD_MIN
    // @DisplayName: FBWT Minimal airspeed
    // @Description: Minimum airspeed for the regular Submode::Fbwa, otherwise Submode::Headhold is activated 
    // @Units: deg
    // @Range: -90 .. 90
    // @Increment: 1
    AP_GROUPINFO("_PITCH_MIN", 3, ModeFBWT, pitch_min, FBWT_PITCH_MIN),

    // // @Param: _AGGR
    // // @DisplayName: Custom FBWT Aggression
    // // @Description: Tuning intensity for controls.
    // // @Values: 0:Gentle, 1:Medium, 2:Aggressive
    // AP_GROUPINFO("_AGGR", 4, ModeFBWT, enable_aggression, 1),

    AP_GROUPEND
};

void ModeFBWT::update()
{
    static bool isDirLocked = false;
    // bool isStall = false;
    RC_Channel *chan;

    // // Limits: Height: >= 100 | 30 m, Airspeed >= 20 (Stall speed)
    // // Pitch > -60 deg;  Nose 30° Down: -30 degrees (or -0.52 radians)
    // // Detect limits violation including stalling  and swich to automatic recovery
    // if(Submode::Fbwa && airspeed)
    // AP::baro().healthy() && AP::baro().get_altitude() >= X
    // AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D && AP::ahrs().groundspeed() >= 3

    // if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
    //     float airspeed_ms = plane.airspeed.get_airspeed();
    //     // Your flight mode logic here
    //      if(airspeed_ms < AS_<MIN && pitch > threshold)
    //          isStall = true;
    // }
    // AP::ahrs().airspeed_estimate(&estimated_airspeed)


    // // Fetch the current pitch from AHRS (returned in radians)
    // float current_pitch_rad = AP::ahrs().get_pitch();
    // // Check if the nose is pointed 30 degrees down or lower
    // if (current_pitch_rad <= DEG_TO_RAD * -30.0f) {
    //     // Your recovery or management logic here
    // }

    // if(isStall) {
    //     pitch_target = negative small;  // If flight height allows
    //     throttle = max;
    //     roll_target = 0;
    // } else {
    //     // Stall exit condition
    //     isStall = false;
    //     restore_mode();
    // }


    // Fix directions by the RC Channel switch
    // Alternative: use plane.g2.dirlock_rcin
    chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DIRLOCK);
    if (chan != nullptr && chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
        static int32_t locked_yaw_cd;  // Locked yaw in centidegrees

        if(!isDirLocked) {
            // Fix directions
            // locked_pitch_cd = plane.nav_pitch_cd;  // ahrs.get_pitch_deg() * 100;
            locked_yaw_cd = plane.nav_controller->nav_bearing_cd();  // AP::ahrs().get_yaw_deg() * 100
            isDirLocked = true;
            const float locked_throttle = plane.channel_throttle->get_control_in() / 45.0f;
            plane.gcs().send_text(MAV_SEVERITY_NOTICE, "FBWA dirlock yaw: %d, pitch: %d, throttle: %u%%",
                wrap_180(int16_t(locked_yaw_cd/100)), int16_t(plane.nav_pitch_cd/100), int8_t(locked_throttle*100));
        }

        // plane.update_load_factor();  // It is likely already called by the main loop, and this one is not strictly necessary
        plane.nav_controller->update_heading_hold(locked_yaw_cd);
        // Pull the resulting 'nav_roll' calculated by the controller and limits to ensure the plane doesn't bank too steeply
        plane.nav_roll_cd = constrain_int32(plane.nav_controller->nav_roll_cd(), -plane.roll_limit_cd, plane.roll_limit_cd);

        // Note: Throttle locking is performed in Plane::set_throttle(void), otherwise the value is set there anyway overwriting the current one
        // // Set fixed throttle
        // SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, locked_throttle);
        return;
    }
    isDirLocked = false;

    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}

void ModeFBWT::run()
{
    // Run base class function and then output throttle
    Mode::run();

    output_pilot_throttle();
}
