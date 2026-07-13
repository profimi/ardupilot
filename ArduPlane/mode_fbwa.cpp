#include "mode.h"
#include "Plane.h"

void ModeFBWA::update()
{
    static bool isDirLocked = false;
    RC_Channel *chan;

    // Fix directions by the RC Channel switch
    // Alternative: use plane.g2.dirlock_rcin
    chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DIRLOCK);
    if (chan != nullptr && chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
        static int32_t locked_yaw_cd;  // Locked yaw in centidegrees
        static int32_t locked_pitch_cd;  // Locked pitch in centidegrees

        if(!isDirLocked) {
            // Fix directions
            locked_pitch_cd = ahrs.get_pitch_deg() * 100;  // Note: we are taking the actual pitch rather than plane.nav_pitch_cd used to to achieve a target altitude or airspeed
            locked_yaw_cd = plane.nav_controller->nav_bearing_cd();  // AP::ahrs().get_yaw_deg() * 100
            isDirLocked = true;
            const float locked_throttle = plane.channel_throttle->get_control_in() / 45.0f;
            plane.gcs().send_text(MAV_SEVERITY_NOTICE, "FBWA dirlock yaw: %d, pitch: %d, throttle: %u%%",
                wrap_180(int16_t(locked_yaw_cd/100)), int16_t(locked_pitch_cd/100), int8_t(locked_throttle*100));
        }

        // plane.update_load_factor();  // It is likely already called by the main loop, and this one is not strictly necessary
        plane.nav_controller->update_heading_hold(locked_yaw_cd);
        // Pull the resulting 'nav_roll' calculated by the controller and limits to ensure the plane doesn't bank too steeply
        plane.nav_roll_cd = constrain_int32(plane.nav_controller->nav_roll_cd(), -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.nav_pitch_cd = locked_pitch_cd;

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

void ModeFBWA::run()
{
    // Run base class function and then output throttle
    Mode::run();

    output_pilot_throttle();
}
