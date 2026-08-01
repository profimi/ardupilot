#include "mode.h"
#include "Plane.h"

// TODO: Check all TODOs, use Arduplane params insted of predefined constants
// No new parameters are used, everythins is derieved from: aparm, TECS, AHR, airspeed (optional_)

// Adrupilot introduces some standard parameters:
// LIM_ROLL_CD (Roll Angle Limit): Sets the maximum allowed bank angle in centidegrees (hundredths of a degree): 4500 is 45 deg
// LIM_PITCH_MAX (Maximum Pitch Up Limit): Sets the maximum allowed nose-up pitch angle in centidegrees: 2500 is 25 deg
// LIM_PITCH_MIN (Maximum Pitch Down Limit): Sets the maximum allowed nose-down pitch angle in centidegrees: -3500 is -35 deg
// TECS_VERT_ACC (Vertical Acceleration Limit) governs how aggressively the autopilot transitions between different vertical speeds.
//  Default: 7.0 m/s² (1.0 .. 10.0), affects all flight modes except MANUAL and ACRO.
// TECS_SINK_MAX (Maximum Descent Rate)  sets the absolute hardest vertical dive rate the autopilot is allowed to initiate during autonomous flight..
//  Default: 5.0 m/s (3 .. 10), affects all flight modes except MANUAL, ACRO and FBWA.
const struct AP_Param::GroupInfo ModeFBWT::var_info[] = {
    // @Param: AIRSPD_MIN
    // @DisplayName: FBWT minimal altitude
    // @Description: Minimum airspeed for the regular Submode::Fbwa, otherwise Submode::Headhold is activated. 
    // @Units: m
    // @Range: 1 255
    // @Increment: 1
    AP_GROUPINFO("ALT_MIN", 1, ModeFBWT, alt_min, FBWT_ALT_MIN),

    // // @Param: AIRSPD_MIN
    // // @DisplayName: FBWT Minimal airspeed
    // // @Description: Minimum airspeed for the regular Submode::Fbwa, otherwise Submode::Headhold is activated 
    // // @Units: deg
    // // @Range: -90 .. 90
    // // @Increment: 1
    // AP_GROUPINFO("PITCH_MIN", 2, ModeFBWT, pitch_min, FBWT_PITCH_MIN),

    // @Param: CTL_EXPOCRV
    // @DisplayName: RC controls exponential smoothing
    // @Description: Use exponential curve smoothing of the RC input controls; 0 means desabled
    // @Range: 0 .. 0.5
    AP_GROUPINFO("CTL_EXPOCRV", 2, ModeFBWT, ctl_expocrv, CTL_EXPOCRV),

    // Note: AIRSPD_MIN is defined automatically as aparm.airspeed_min + (aparm.airspeed_cruise - aparm.airspeed_min) / 10.f
    // // @Param: AIRSPD_MIN
    // // @DisplayName: FBWT minimal airspeed (recommended: 1.2f * stall_speed)
    // // @Description: Minimum airspeed for the regular Submode::Fbwa, otherwise Submode::Headhold is activated 
    // // @Units: m/s
    // // @Range: 10 40
    // // @Increment: 1
    // AP_GROUPINFO("AIRSPD_MIN", 3, ModeFBWT, airspd_min, FBWT_AIRSPD_MIN),

    // // @Param: AGGR
    // // @DisplayName: Custom FBWT Aggression
    // // @Description: Tuning intensity for controls.
    // // @Values: 0:Gentle, 1:Medium, 2:Aggressive
    // AP_GROUPINFO("AGGR", 4, ModeFBWT, enable_aggression, 1),

    AP_GROUPEND
};


// Helpers
static float constrain_symmetric(float val, float limit)
{
    return constrain_float(val, -limit, limit);
}

static float blend_limits(float prev, float target, float alpha)
{
    return prev + alpha * (target - prev);
}

// static float estimate_load_factor_cd(float roll_cd)
// {
//     float phi = radians(roll_cd * 0.01f);
//     return 1.0f / MAX(cosf(phi), 0.1f);
// }

static float estimate_load_factor(float roll_deg)
{
    float phi = radians(roll_deg);
    return 1.0f / MAX(cosf(phi), 0.1f);
}

// Sidesleep (beta) estimatio
float ModeFBWT::estimate_beta()
{
    const Vector3f &vel = plane.ahrs.get_velocity_NED();

    if (!plane.ahrs.airspeed_sensor_enabled()) {
        return 0.0f; // fallback
    }

    float airspeed = plane.airspeed_estimate();

    if (airspeed < 5.0f) {
        return 0.0f;
    }

    // lateral velocity approximation
    float v_lat = vel.y;

    float beta = atanf(v_lat / airspeed);

    return beta;
}

// Dynamic stall model (CL-based)
void ModeFBWT::apply_dynamic_stall(float &pitch_cmd, float Nz)
{
    // Approximate lift coefficient demand
    float CL = Nz;

    const float CL_max = 1.5f; // conservative default

    if (CL > CL_max && pitch_cmd > 0) {
        float scale = CL_max / CL;
        pitch_cmd *= scale;
    }
}

// AoA and Stall Protection
void ModeFBWT::apply_stall_protection(float &pitch_cmd, float airspeed, float Nz)
{
    float aoa = plane.ahrs.getAOA();
    bool aoa_valid = isfinite(aoa);

    if (aoa_valid) {
        const float aoa_lim = radians(15.0f);

        if (aoa > aoa_lim && pitch_cmd > 0) {
            float scale = constrain_float((aoa_lim - aoa)/aoa_lim, 0.0f, 1.0f);
            pitch_cmd *= scale;
        }
        return;
    }

    float vmin = plane.aparm.airspeed_min;
    const float margin = 1.3f;

    if (airspeed > 0 && airspeed < vmin * margin) {
        float scale = airspeed / (vmin * margin);

        if (pitch_cmd > 0) pitch_cmd *= scale;
        if (Nz > 1.0f) pitch_cmd *= (1.0f / Nz);
    }
}

// Adaptive G-Limit
float ModeFBWT::compute_adaptive_nz_limit(float airspeed)
{
    const float nz_high = 2.5f;  // conservative structural limit
    const float nz_low  = 1.2f;

    if (airspeed <= 0)
        return 2.0f;

    float vmin = plane.aparm.airspeed_min;
    float vmax = plane.aparm.airspeed_max;

    float t = constrain_float((airspeed - vmin) / (vmax - vmin), 0.0f, 1.0f);

    return nz_low + t * (nz_high - nz_low);
}

// Load factor NZ limiter with adaptive limit
void ModeFBWT::apply_nz_limit(float &roll_cmd, float airspeed)
{
    float nz_lim = compute_adaptive_nz_limit(airspeed);

    float Nz = estimate_load_factor(roll_cmd);

    if (Nz > nz_lim) {
        float phi = acosf(1.0f / nz_lim);
        roll_cmd = constrain_symmetric(roll_cmd, degrees(phi));
    }
}

// Energy-aware roll limiting
void ModeFBWT::apply_energy_roll_limit(float &roll_cmd)
{
    AP_TECS *tecs = plane.tecs;
    if (!tecs) return;

    float STEdot = tecs->get_SPDot();

    if (STEdot < -3.0f) {
        float scale = constrain_float((STEdot + 5.0f) / 2.0f, 0.3f, 1.0f);
        roll_cmd *= scale;
    }
}

// Terrain-aware protection
void ModeFBWT::apply_terrain_protection(float &pitch_cmd)
{
    float rel_alt = plane.relative_altitude;

    if (rel_alt < 20.0f) {
        float scale = constrain_float(rel_alt / 20.0f, 0.3f, 1.0f);

        // Reduce aggressive pitch-up near ground
        if (pitch_cmd > 0) {
            pitch_cmd *= scale;
        }
    }
}

// TECS energy limiter
void ModeFBWT::apply_energy_limiter(float &pitch_cmd)
{
    AP_TECS *tecs = plane.tecs;
    if (!tecs)
        return;

    float STEdot = tecs->get_SPDot();

    const float sink_limit = -5.0f;  // m/s
    const float climb_limit = 5.0f;  // TODO: Load from Arduplane params

    if (STEdot < sink_limit && pitch_cmd > 0) {
        float scale = constrain_float((STEdot - sink_limit) / sink_limit, 0.0f, 1.0f);
        pitch_cmd *= scale;
        // Or just:  pitch_cmd *= 0.5f;
    }

    if (STEdot > climb_limit && pitch_cmd < 0) {
        float scale = constrain_float((climb_limit - STEdot) / climb_limit, 0.0f, 1.0f);
        pitch_cmd *= scale;
        // Or just:  pitch_cmd *= 0.5f;
    }
}

// Turn coordination (beta-based)
void ModeFBWT::apply_turn_coordination(float roll_cmd)
{
    float beta = estimate_beta();

    float rudder = constrain_float(-beta * 2.0f, -1.0f, 1.0f);

    plane.channel_rudder->set_servo_out(rudder * 4500);
}

// Combined envelope limiter
void ModeFBWT::apply_envelope_limits(float &roll_cmd, float &pitch_cmd)
{
    float airspeed = plane.airspeed_estimate();
    float Nz = estimate_load_factor(roll_cmd);

    apply_nz_limit(roll_cmd, airspeed);
    apply_energy_roll_limit(roll_cmd);

    apply_stall_protection(pitch_cmd, airspeed, Nz);
    apply_dynamic_stall(pitch_cmd, Nz);
    apply_energy_limiter(pitch_cmd);
    apply_terrain_protection(pitch_cmd);
}

// Main function
void ModeFBWT::update()
{
    float pitch_in = plane.channel_pitch->norm_input();
    float roll_in  = plane.channel_roll->norm_input();

    float pitch_max = plane.aparm.pitch_limit_max;
    float pitch_min = plane.aparm.pitch_limit_min;
    float roll_max  = plane.aparm.roll_limit_cd * 0.01f;

    float pitch_cmd = pitch_in * (pitch_in > 0 ? pitch_max : -pitch_min);
    float roll_cmd  = roll_in  * roll_max;

    switch (_submode) {

    case FBWT_SUBMODE_FBWA:
        break;

    case FBWT_SUBMODE_HEADHOLD:
        if (fabsf(roll_in) < 0.05f) {
            roll_cmd = plane.nav_roll_cd * 0.01f;
        }
        break;

    case FBWT_SUBMODE_LEVELUP:
        if (pitch_in > 0.8f) {
            pitch_cmd = pitch_max;
            roll_cmd = blend_limits(roll_cmd, 0.0f, 0.2f);
        }
        break;
    }

    apply_envelope_limits(roll_cmd, pitch_cmd);

    plane.nav_pitch_cd = pitch_cmd * 100.0f;
    plane.nav_roll_cd  = roll_cmd  * 100.0f;

    apply_turn_coordination(roll_cmd);

    if (plane.g.log_bitmask & MASK_LOG_ATTITUDE_FAST) {
        AP::logger().Write(
            "FBWT",
            "TimeUS,Rin,Pin,Rcmd,Pcmd,Nz,Spd",
            "Qffffff",
            AP_HAL::micros64(),
            roll_in,
            pitch_in,
            roll_cmd,
            pitch_cmd,
            estimate_load_factor(roll_cmd),
            plane.airspeed_estimate()
        );
    }
}

// void ModeFBWT::update()
// {
//     update_submode();

//     // // --- 1. Pilot inputs (normalized), row stick positions
//     float pitch_in = plane.channel_pitch->norm_input();  // -1..1
//     float roll_in  = plane.channel_roll->norm_input();
//     // Stick shaping (more natural feel)
//     if(ctl_expocrv) {
//         pitch_in = expo_curve(pitch_in, ctl_expocrv);
//         roll_in = expo_curve(pitch_in, ctl_expocrv);
//     }
    
//     // --- 5. Convert pilot input → demanded angles
//     const float pitch_max = plane.aparm.pitch_limit_max_deg;
//     // const float pitch_min = plane.aparm.pitch_limit_min_deg;
//     const float roll_max  = plane.aparm.roll_limit_deg;
//     float pitch_cmd = pitch_in * pitch_max;
//     float roll_cmd  = roll_in  * roll_max;

//     float pitch_cmd = plane.nav_pitch_cd * 0.01f; // degrees from centidegrees
//     float roll_cmd  = plane.nav_roll_cd  * 0.01f;

//     apply_envelope_limits(pitch_in, roll_in);  //     void ModeFBWT::apply_envelope_limits(float &pitch, float &roll);

//     // Send to attitude controller
//     plane.nav_pitch_cd = pitch_cmd * 100.0f;
//     plane.nav_roll_cd  = roll_cmd  * 100.0f;
// }

// void ModeFBWT::apply_envelope_limits(float &pitch_cmd, float &roll_cmd)
// {
//     // --- 2. Get configured limits (degrees)
//     const float pitch_max = plane.aparm.pitch_limit_max_deg;
//     const float pitch_min = plane.aparm.pitch_limit_min_deg;
//     const float roll_max  = plane.aparm.roll_limit_deg;

//     // --- 3. Energy-based limiter (0.3 .. 1.0)
//     // Prevent over limiting via MAX
//     float e_lim = MAX(compute_energy_limiter(), 0.4f);

//     // --- 4. Scale limits based on energy
//     float pitch_max_eff = pitch_max * e_lim;
//     float pitch_min_eff = pitch_min * e_lim;
//     // float roll_max_eff  =  roll_max  * (0.5f + 0.5f * e_lim);
//     // Stronger roll limiting at low energy
//     float roll_max_eff  = roll_max  * sqrt(e_lim);
//     // roll is less aggressively limited than pitch

//     // --- 5. Convert pilot input → demanded angles
//     pitch_cmd = pitch_cmd * pitch_max;
//     roll_cmd  = roll_cmd  * roll_max;

//     // --- 6. Apply envelope limits
//     pitch_cmd = constrain_float(pitch_cmd, pitch_min_eff, pitch_max_eff);
//     roll_cmd  = constrain_float(roll_cmd, -roll_max_eff, roll_max_eff);

//     // --- 7. Optional: pitch protection when banked (approx load factor)
//     float bank_rad = radians(fabsf(roll_cmd));
//     float load_factor = 1.0f / MAX(cosf(bank_rad), 0.5f);  // avoid div by small

//     // reduce pitch authority at high bank (stall prevention)
//     float pitch_bank_scale = 1.0f / load_factor;
//     pitch_cmd *= constrain_float(pitch_bank_scale, 0.5f, 1.0f);  // Or 0.6 .. 1 for a softer protection
// }

// FBWTPhase ModeFBWT::detect_phase() const
// {
//     float pitch = ahrs.pitch;
//     float vz    = get_vertical_speed();   // baro or derived
//     float thr   = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
//     float load  = get_load_factor();

//     if (thr > 0.8f && vz > 1.0f) {
//         return PHASE_TAKEOFF;
//     }

//     if (vz > 0.5f) {
//         return PHASE_CLIMB;
//     }

//     if (vz < -0.5f && thr < 0.3f) {
//         return PHASE_DESCENT;
//     }

//     if (fabsf(load) > 1.3f) {
//         return PHASE_MANEUVER;
//     }

//     if (thr < 0.3f && pitch < radians(5)) {
//         return PHASE_LANDING;
//     }

//     return PHASE_CRUISE;
// }

// ModeFBWT::EnvelopeLimits ModeFBWT::get_phase_limits(FBWTPhase phase) const
// {
//     switch (phase) {

//     case PHASE_TAKEOFF:
//         return { 
//             radians(10),   // conservative AoA
//             1.5f,          // low G
//             0.9f,          // high energy requirement
//             1.2f,
//             0.5f           // strong protection
//         };

//     case PHASE_CLIMB:
//         return {
//             radians(12),
//             2.0f,
//             0.8f,
//             1.3f,
//             0.4f
//         };

//     case PHASE_CRUISE:
//         return {
//             radians(14),
//             2.5f,
//             0.7f,
//             1.5f,
//             0.3f
//         };

//     case PHASE_MANEUVER:
//         return {
//             radians(16),   // allow higher AoA
//             3.5f,          // higher G
//             0.6f,
//             1.6f,
//             0.3f
//         };

//     case PHASE_DESCENT:
//         return {
//             radians(12),
//             2.0f,
//             0.6f,
//             1.4f,
//             0.3f
//         };

//     case PHASE_LANDING:
//         return {
//             radians(10),   // very protective
//             1.5f,
//             0.85f,
//             1.2f,
//             0.5f
//         };
//     }

//     return get_phase_limits(PHASE_CRUISE);
// }

// float ModeFBWT::compute_envelope_limiter() const
// {
//     FBWTPhase phase = detect_phase();
//     // TODO: Blend phases smoothly
//     // limits = blend_limits(previous_phase, new_phase, alpha);
//     EnvelopeLimits L = get_phase_limits(phase);

//     float aoa = get_aoa();
//     float load = get_load_factor();
//     float E = compute_energy_normalized(); // normalized 0..~2

//     // AoA factor
//     float f_aoa = 1.0f;
//     if (aoa > 0.0f) {
//         float margin = L.aoa_max - aoa;
//         f_aoa = (margin <= 0.0f) ? L.limiter_floor :
//                 constrain_float(margin / L.aoa_max,
//                                 L.limiter_floor, 1.0f);
//     }

//     // V-n factor
//     float f_vn = (load <= L.n_max) ? 1.0f :
//         constrain_float(L.n_max / load,
//                         L.limiter_floor, 1.0f);

//     // Energy factor
//     float f_e = 1.0f;
//     if (E < L.energy_min) {
//         f_e = constrain_float(E / L.energy_min,
//                               L.limiter_floor, 1.0f);
//     } else if (E > L.energy_max) {
//         f_e = constrain_float(L.energy_max / E,
//                               L.limiter_floor, 1.0f);
//     }

//     // Final unified limiter
//     return MIN(f_aoa, MIN(f_vn, f_e));
// }





// // Aurion version with extra manual parameters (redundant)
// // void ModeFBWT::update()
// // {
// //     // === INPUTS ===
// //     float pitch_cmd = plane.channel_pitch->get_control_in();   // pilot input
// //     float roll_cmd  = plane.channel_roll->get_control_in();

// //     float dt = AP::scheduler().get_loop_period_s();

// //     // === AIRCRAFT PARAMETERS ===
// //     const float mass      = plane.aparm.mass;
// //     const float S         = plane.aparm.wing_area;
// //     const float CL_max    = plane.aparm.CL_max;
// //     const float CL_alpha  = plane.aparm.CL_alpha;

// //     const float n_struct_max = plane.aparm.n_max;
// //     const float n_struct_min = plane.aparm.n_min;

// //     // === STATE ESTIMATION ===
// //     float V    = MAX(cur_airspeed, 5.0f);     // avoid divide-by-zero
// //     float rho  = plane.get_air_density();
// //     float nz   = plane.get_load_factor();

// //     // === AoA ===
// //     float alpha = get_aoa();

// //     // === LIFT COEFFICIENT ===
// //     float CL = CL_alpha * alpha;
// //     CL = constrain_float(CL, -CL_max, CL_max);

// //     // ============================================================
// //     // 1. AoA PROTECTION (Primary Stall Protection)
// //     // ============================================================
// //     float alpha_max = plane.aparm.alpha_max;
// //     float alpha_min = plane.aparm.alpha_min;

// //     float aoa_factor = 1.0f;

// //     if (alpha > alpha_max) {
// //         aoa_factor = constrain_float(1.0f - (alpha - alpha_max) * 5.0f, 0.0f, 1.0f);
// //     }
// //     if (alpha < alpha_min) {
// //         aoa_factor = constrain_float(1.0f - (alpha_min - alpha) * 5.0f, 0.0f, 1.0f);
// //     }

// //     // ============================================================
// //     // 2. TRUE DYNAMIC V–n LIMIT (CL-based)
// //     // ============================================================
// //     float n_stall = 2.f;  // Conservative fixed limit, fallback for the reduced envelope
    
// //     if(has_airspeed)
// //         n_stall = (0.5f * rho * V * V * S * CL_max) / (mass * GRAVITY_MSS);

// //     float n_max = MIN(n_struct_max, n_stall);
// //     float n_min = MAX(n_struct_min, -n_stall);

// //     float g_factor = 1.0f;

// //     if (nz > n_max) {
// //         g_factor = constrain_float(1.0f - (nz - n_max) * 0.5f, 0.0f, 1.0f);
// //     }
// //     if (nz < n_min) {
// //         g_factor = constrain_float(1.0f - (n_min - nz) * 0.5f, 0.0f, 1.0f);
// //     }

// //     // ============================================================
// //     // 3. ENERGY PROTECTION (Total Energy Control)
// //     // ============================================================
// //     float h  = plane.get_altitude();
// //     float dh = plane.get_climb_rate();

// //     float E     = compute_energy();  // GRAVITY_MSS * h + 0.5f * V * V;
// //     float E_dot = GRAVITY_MSS * dh + V * plane.get_airspeed_rate();

// //     float E_min = plane.aparm.energy_min;
// //     float E_max = plane.aparm.energy_max;

// //     float energy_factor = 1.0f;

// //     if (E < E_min && pitch_cmd > 0)
// //         energy_factor = constrain_float((E - E_min) / E_min, 0.0f, 1.0f);
// //     if (E > E_max && pitch_cmd < 0)
// //         energy_factor = constrain_float((E_max - E) / E_max, 0.0f, 1.0f);

// //     // ============================================================
// //     // 4. COMBINED LIMITER (THE CORE)
// //     // ============================================================
// //     float limit_factor = MIN(aoa_factor, MIN(g_factor, energy_factor));

// //     pitch_cmd *= limit_factor;

// //     // ============================================================
// //     // 5. OPTIONAL SOFT RECOVERY BIAS
// //     // ============================================================
// //     if (aoa_factor < 0.5f) {
// //         pitch_cmd -= 0.3f * (alpha - alpha_max);  // nose-down push
// //     }

// //     // Hard fallback
// //     if(!has_gps && !has_velocity && !has_airspeed) {
// //         roll_cmd = constrain_float(roll_cmd, radians(-45), radians(45));  // 0
// //         pitch_cmd = constrain_float(pitch_cmd, radians(-20), radians(20));
// //     }

// //     // ============================================================
// //     // 6. APPLY CONTROLS
// //     // ============================================================
// //     plane.pitchController.set_input(pitch_cmd);
// //     plane.rollController.set_input(roll_cmd);
// // }

// // // Claude version
// // void ModeFBWT::update()
// // {
// //     has_gps      = plane.ahrs.have_position();
// //     has_velocity = plane.ahrs.have_velocity();
// //     has_airspeed = plane.airspeed_sensor.enabled() && plane.airspeed > 5.0f;

// //     if(has_airspeed)
// //         cur_airspeed = plane.airspeed;
// //     else if (has_velocity)
// //         cur_airspeed = plane.groundspeed();
// //     else cur_airspeed = (plane.aparm.airspeed_min.get() + plane.aparm.airspeed_cruise.get()) / 2.f;  // m/s

// //     // 1. Run submode logic
// //     update_submode();

// //     // 2. Get commands generated by submode
// //     float pitch_cmd = _pitch_cmd;
// //     float roll_cmd  = _roll_cmd;

// //     // 3. Apply NEW unified protection
// //     apply_envelope_protection(pitch_cmd);

// //     // 4. Final fallback clamp
// //     pitch_cmd = constrain_float(pitch_cmd, pitch_min, pitch_max);

// //     // 5. Send to controllers
// //     plane.pitchController.set_input(pitch_cmd);
// //     plane.rollController.set_input(roll_cmd);
// // }

// // void ModeFBWT::update_submode()
// // {
// //     float alpha = compute_alpha();
// //     float alt = plane.relative_altitude;

// //     bool stall = alpha > alpha_limit;
// //     bool low_alt = alt < alt_min;

// //     switch (submode) {

// //     case Submode::Fbwa:
// //         if (stall || low_alt) {
// //             submode = Submode::Levelup;
// //             assist_gain = 1.0f;
// //         }
// //         break;

// //     case Submode::Levelup:
// //         if (alt > alt_min + alt_hyst) {
// //             submode = Submode::Headhold;
// //             target_alt = alt;
// //             target_heading = plane.ahrs.yaw;
// //         }
// //         break;

// //     case Submode::Headhold:
// //         assist_gain *= assist_decay;
// //         if (assist_gain < 0.2f) {
// //             submode = Submode::Fbwa;
// //         }
// //         break;
// //     }
// // }

// // float ModeFBWT::compute_alpha()
// // {
// //     return get_aoa();
// //     // fallback estimate
// //     float vz = 0;
// //     if(plane.ahrs.have_velocity())
// //         vz = -ahrs.get_velocity_NED().z;
// //     return plane.ahrs.pitch - atan2f(vz, cur_airspeed);
// // }

// // float ModeFBWT::get_aoa() const
// // {
// //     // Fallback: synthetic AoA approximation
// //     float pitch = degrees(plane.ahrs.pitch);
// //     float flight_path_angle = get_flight_path_angle();

// //     return pitch - flight_path_angle;
// // }

// // float ModeFBWT::get_flight_path_angle() const
// // {
// //     if (plane.ahrs.have_velocity()) {
// //         Vector3f vel = plane.ahrs.get_velocity_NED();
// //         float horizontal_speed = sqrtf(sq(vel.x) + sq(vel.y));

// //         if (horizontal_speed > 1.0f)
// //             return degrees(atan2f(-vel.z, horizontal_speed));
// //     }

// //     return 0.0f; // fallback
// // }

// // void ModeFBWT::apply_envelope(float &pitch, float &roll)
// // {
// //     float alpha = compute_alpha();
// //     float nz = get_load_factor();

// //     float stall_f = stall_factor(alpha);  // AoA
// //     float g_f     = g_limit_factor(nz);
// //     float e_f     = energy_factor();  // Energy

// //     float limit = MIN(MIN(stall_f, g_f), e_f);

// //     pitch *= limit;
// //     roll *= limit;

// //     // // Optional recovery bias
// //     // if (stall_f < 0.5f) {
// //     //     pitch_cmd -= 0.3f * get_aoa_error();  // * (alpha - alpha_limit)
// //     // }

// //     // 4. Final fallback clamp
// //     const static float pitch_lim_rad = radians((float) -pitch_min);  // 50-60 deg
// //     const static float roll_max_rad = radians(50.f);  // 45-50
// //     pitch = constrain_float(pitch, -pitch_lim_rad, pitch_lim_rad * 0.8f);
// //     roll = constrain_float(roll, -roll_max_rad, roll_max_rad);
// // }

// // float ModeFBWT::stall_factor(float alpha)
// // {
// //     if (alpha < alpha_limit) return 1.0f;

// //     float excess = alpha - alpha_limit;
// //     return constrain_float(1.0f - excess * 5.0f, 0.0f, 1.0f);
// // }

// // float ModeFBWT::g_limit_factor(float nz)
// // {
// //     if (nz > nz_max) {
// //         return constrain_float(1.0f - (nz - nz_max) * 0.5f, 0.0f, 1.0f);
// //     }
// //     if (nz < nz_min) {
// //         return constrain_float(1.0f - (nz_min - nz) * 0.5f, 0.0f, 1.0f);
// //     }
// //     return 1.0f;
// // }

// // --- Helpers ---
// //  True if the AHRS currently has a usable horizontal+vertical position
// //  estimate. Checked via the AHRS abstraction (not the GPS driver
// //  directly) so this degrades correctly regardless of *why* position is
// //  unavailable, and is the single gate used everywhere in this file that
// //  would otherwise need GPS
// bool ModeFBWT::have_position() const
// {
//     Location loc;
//     return plane.ahrs.get_position(loc);
// }

// float ModeFBWT::get_load_factor() const
// {
//     return plane.ahrs.get_accel().z / GRAVITY_MSS;
// }

// // Universal energy limiter
// float ModeFBWT::compute_energy_limiter() const
// {
//     float f_rate = energy_factor();          // fast
//     float f_abs  = compute_energy_factor();  // slow

//     float w = get_phase_weight(); // 0..1

//     // blend (phase dependent)
//     return constrain_float(
//         w * f_rate + (1.0f - w) * f_abs,
//         0.2f, 1.0f  // min: 0.2f .. 0.4f
//     );
// }

// float ModeFBWT::get_phase_weight()
// {
//     switch(submode) {
//     case Submode::Levelup:  // Climb
//         return 0.8f;   // favor rate
//     case Submode::Headhold: 
//         return 0.2f;   // favor absolute
//     case Submode::Fbwa:   // Maneuvering
//         return 0.9f;   // almost all rate
//     }
//     return 0.5f; // fallback
// }

// // Fast, Energy rate, noisy
// float ModeFBWT::energy_factor()
// {
//     float e_rate = get_energy_rate();

//     if (e_rate > -0.5f) return 1.0f;

//     return constrain_float(1.0f + e_rate, 0.3f, 1.0f);
// }

// // // float ModeFBWT::get_energy_rate()
// // // {
// // //     float vz = 0;
// // //     if(plane.ahrs.have_velocity())
// // //         vz = -ahrs.get_velocity_NED().z;
// // //     float v  = cur_airspeed;
// // //
// // //     return vz + (v * plane.airspeed_rate) / GRAVITY_MSS;
// // // }

// // float ModeFBWT::get_energy_rate()
// // {
// //     // --- 1. Vertical speed (positive up)
// //     float vz = 0.0f;
// //     if (has_velocity)
// //         vz = -ahrs.get_velocity_NED().z;

// //     // --- 2. Airspeed estimate
// //     float v = cur_airspeed;  // get_airspeed_estimate();

// //     // --- 3. Compute dV/dt with filtering
// //     static float v_prev = 0.0f;
// //     static float dv_filtered = 0.0f;
// //     static uint32_t t_prev = 0;

// //     uint32_t now = AP_HAL::millis();
// //     float dt = (now - t_prev) * 0.001f;

// //     float dv = 0.0f;

// //     if (dt > 0.01f && dt < 0.2f) {   // valid timing window; 0.2f..0.5f
// //         dv = (v - v_prev) / dt;

// //         // low-pass filter (critical!)
// //         const float alpha = 0.2f;
// //         dv_filtered = alpha * dv + (1.0f - alpha) * dv_filtered;
// //     }

// //     v_prev = v;
// //     t_prev = now;

// //     // --- 4. Energy rate
// //     return vz + (v * dv_filtered) / GRAVITY_MSS;
// // }

// float ModeFBWT::get_energy_rate()
// {
//     // --- 1. Vertical speed (m/s, positive up)
//     float vz = 0.0f;

//     if (plane.ahrs.have_velocity()) {
//         const Vector3f &vel = plane.ahrs.get_velocity_NED();
//         vz = -vel.z;  // NED frame: down is positive
//     }

//     // --- 2. Speed estimate (m/s)
//     float v = cur_airspeed;

//     // --- 3. Forward acceleration (body X axis)
//     float ax = 0.0f;

//     const AP_InertialSensor &ins = AP::ins();
//     if (ins.get_accel_count() > 0)
//         ax = ins.get_accel(0).x;  // body-frame forward accel

//     // --- 4. Energy rate
//     // return vz + (v * ax) / GRAVITY_MSS;

//     static float ax_filt = 0.0f;
//     const float alpha = 0.2f;

//     ax_filt = alpha * ax + (1.0f - alpha) * ax_filt;

//     return vz + (v * ax_filt) / GRAVITY_MSS;
// }

// float ModeFBWT::get_energy_rate() {
//     float ax = ahrs.get_accel_body().x;
//     return vz + (v * ax) / GRAVITY_MSS;
// }

// // float ModeFBWT::get_energy_rate()
// // {
// //     return plane.TECS.get_SPE_rate() +
// //            plane.TECS.get_SKE_rate();
// // }

// // Slow, total energy
// float ModeFBWT::compute_energy_factor() const
// {
//     float E = compute_energy();

//     // Can be calculated automatically from the min speed parameter and min/max flight height
//     // TODO: replace Emin/maxvalues, evaluating parameters from the config
//     // Takeoff_Alt * g + SPEED_MIN^2/2
//     constexpr float Emin = 250;  // plane.g.fbwt_energy_min;  // 250..500 = 9,8*h + V^2/2  ~ 16 m/s at 12 m
//     constexpr float Emax = 5000;  // plane.g.fbwt_energy_max;  // 1600..5000  // ~ 25 m/s at 500 m

//     // const float min_factor = 0.2f + 0.2f * fabsf(pitch);
//     if (E < Emin)
//         return constrain_float(E / Emin, 0.2f, 1.0f);  // min: 0, 0.2 .. 0.4

//     if (E > Emax)
//         return constrain_float((Emax / E), 0.2f, 1.0f);

//     return 1.0f;
// }

// float ModeFBWT::compute_energy() const
// {
//     float h = plane.relative_altitude;

//     float V;
//     if (plane.airspeed_sensor.enabled())
//         V = plane.airspeed;
//     else if (plane.ahrs.have_velocity())
//         V = plane.groundspeed();
//     else V = 15.0f; // fallback

//     return GRAVITY_MSS * h + 0.5f * V * V;
// }

// void ModeFBWT::run_fbwa()
// {
//     float pitch = plane.channel_pitch->get_control_in();
//     float roll  = plane.channel_roll->get_control_in();

//     float alpha = compute_alpha();
//     float nz = get_load_factor();

//     float stall_f = stall_factor(alpha);
//     float g_f = g_limit_factor(nz);
//     float e_f = energy_factor();

//     float limit = MIN(MIN(stall_f, g_f), e_f);

//     pitch *= limit;
//     roll  *= limit;

//     plane.nav_pitch_cd = pitch * 4500;
//     plane.nav_roll_cd  = roll  * 4500;

//     plane.calc_throttle();

// //         // Fix directions by the RC Channel switch
// //         // Alternative: use plane.g2.dirlock_rcin
// //         chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DIRLOCK);
// //         if (chan != nullptr && chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
// //             headhold(isDirLocked, true);
// //             return;
// //         }
// //         isDirLocked = false;

// //         // set nav_roll and nav_pitch using sticks
// //         plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
// //         plane.update_load_factor();
// //         float pitch_input = plane.channel_pitch->norm_input();
// //         if (pitch_input > 0) {
// //             plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
// //         } else {
// //             plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
// //         }
// //         plane.adjust_nav_pitch_throttle();
// //         plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
// //         if (plane.fly_inverted()) {
// //             plane.nav_pitch_cd = -plane.nav_pitch_cd;
// //         }

// //         if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
// //             // FBWA failsafe glide
// //             plane.nav_roll_cd = 0;
// //             plane.nav_pitch_cd = 0;
// //             SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
// //         }
// //         chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
// //         if (chan != nullptr) {
// //             // check for the user enabling FBWA taildrag takeoff mode
// //             bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
// //             if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
// //                 if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
// //                     plane.auto_state.fbwa_tdrag_takeoff_mode = true;
// //                     plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
// //                 }
// //             }
// //         }

// }

// void ModeFBWT::run_levelup()
// {
//     float pitch = radians(10.0f); // nose up gently
//     float roll  = 0;

//     apply_envelope(pitch, roll);

//     plane.nav_pitch_cd = degrees(pitch) * 100;
//     plane.nav_roll_cd  = 0;

//     plane.throttle_set(1.0f); // full throttle
// }

// void ModeFBWT::run_headhold()
// {
//     float roll;
//     if (has_gps && has_velocity) {
//         // CASE 1: Full Navigation (GPS + velocity) → Use L1
//         // L1 lateral control
//         plane.nav_controller->update_waypoint(
//             plane.current_loc,
//             plane.current_loc.offset_bearing(target_heading, 100)
//         );
//         roll = plane.nav_roll_cd * 0.01f;
//     } else {
//         // CASE 2: No GPS → Heading Hold using yaw/heading
//         float current_heading = degrees(plane.ahrs.yaw);
//         float error = wrap_180(target_heading - current_heading);

//         // simple P controller (can be PI if needed)
//         const float Kp_heading = 0.05f;

//         float roll_target = constrain_float(Kp_heading * error, -radians(45), radians(45));
//         // roll = roll_target;
//         // Consider yaw rate damping for smoother fallback
//         float yaw_rate = plane.ahrs.get_gyro().z;
//         roll -= 0.02f * yaw_rate;
//     }

//     // TECS altitude hold
//     plane.tecs_controller->update_pitch_throttle(
//         target_alt,
//         cur_airspeed,
//         plane.groundspeed()
//     );

//     float pitch = plane.nav_pitch_cd * 0.01f;

//     // Envelope protection
//     apply_envelope(pitch, roll);

//     // Assist blending
//     float pilot_pitch = plane.channel_pitch->get_control_in();
//     float pilot_roll  = plane.channel_roll->get_control_in();

//     pitch = assist_gain * pitch + (1 - assist_gain) * pilot_pitch;
//     roll  = assist_gain * roll  + (1 - assist_gain) * pilot_roll;

//     plane.nav_pitch_cd = pitch * 4500;
//     plane.nav_roll_cd  = roll  * 4500;
// }

// void ModeFBWT::run()
// {
//     // float pitch_cmd = 0;
//     // float roll_cmd  = 0;

//     // --- Submode behavior ---
//     switch (submode) {
//     case Submode::Levelup:
//         run_levelup();
//         break;
//     case Submode::Headhold:
//         run_headhold();
//         break;
//     case Submode::Fbwa:
//     default:
//         run_fbwa();
//         break;
//     }
// }

// // ModeFBWT::ModeFBWT(): isDirLocked{false}, submode{Submode::Fbwa}, alt_max{0}, airspd_max{0}
// //     , airspd_min(roundf(aparm.airspeed_min + (aparm.airspeed_cruise - aparm.airspeed_min) / 10.f))
// // {}

// // // TODO: complete pitch  when necessary
// // void ModeFBWT::headhold(bool &isDirLocked, bool doPitchLock)
// // {
// //     static int32_t locked_yaw_cd;  // Locked yaw in centidegrees
// //     static int32_t locked_pitch_cd;  // Locked pitch in centidegrees

// //     if(!isDirLocked) {
// //         // Fix directions
// //         locked_pitch_cd = ahrs.get_pitch_deg() * 100;  // Note: we are taking the actual pitch rather than plane.nav_pitch_cd used to to achieve a target altitude or airspeed
// //         locked_yaw_cd = plane.nav_controller->nav_bearing_cd();  // AP::ahrs().get_yaw_deg() * 100
// //         isDirLocked = true;
// //         const float locked_throttle = plane.channel_throttle->get_control_in() / 45.0f;
// //         plane.gcs().send_text(MAV_SEVERITY_NOTICE, "FBWA dirlock yaw: %d, pitch: %d, throttle: %u%%",
// //             wrap_180(int16_t(locked_yaw_cd/100)), int16_t(locked_pitch_cd/100), int8_t(locked_throttle*100));
// //     }

// //     // plane.update_load_factor();  // It is likely already called by the main loop, and this one is not strictly necessary
// //     plane.nav_controller->update_heading_hold(locked_yaw_cd);
// //     // Pull the resulting 'nav_roll' calculated by the controller and limits to ensure the plane doesn't bank too steeply
// //     plane.nav_roll_cd = constrain_int32(plane.nav_controller->nav_roll_cd(), -plane.roll_limit_cd, plane.roll_limit_cd);
// //     plane.nav_pitch_cd = locked_pitch_cd;

// //     // Note: Throttle locking is performed in Plane::set_throttle(void), otherwise the value is set there anyway overwriting the current one
// //     // // Set fixed throttle
// //     // SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, locked_throttle);
// // }

// bool ModeFBWT::_enter()
// {
// #if HAL_SOARING_ENABLED
//     // for ArduSoar soaring_controller
//     plane.g2.soaring_controller.init_cruising();
// #endif

//     if (!AP::ahrs().healthy()) {
//         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: AHRS is not healthy, FBWT is unstable");
//         // Note: It makes sense to allow FBWC even for unhealthy AHRS, moving the responsibility to pilot
//         // return false;
//     }

//     // plane.set_target_altitude_current();
//     // target_yaw = AP::ahrs().get_yaw_rad();

//     // Resent the submode toe the FBWA like
//     submode = Submode::Fbwa;
//     // isDirLocked = false;

//     return true;
// }

// // void ModeFBWT::update()
// // {
// //     static bool isDirLocked = false;
// //     // bool isStall = false;
// //     RC_Channel *chan;
// //     // uint8_t airspd = 0;  // Current airspeed
// //     uint16_t alt = 0;  // Current altitude
// //     float taspd;  // True airspeed

// //     // float plane_tas_min = plane.aparm.airspeed_min * plane.ahrs.get_EAS2TAS();  // EAS2TAS 
// //     // if (plane.aparm.stall_prevention) {
// //     //     // Replicates the TECS internal adjustment: _TASmin *= load_factor
// //     //     plane_tas_min *= plane.get_load_factor(); 
// //     // }    

// //     // // aparm.airspeed_stall < aparm.airspeed_min < aparm.airspeed_cruise
// //     // if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
// //     //     airspd = constrain_float(plane.airspeed.get_airspeed(), 0, 0xFF);
// //     //     if(airspd_max < airspd)
// //     //         airspd_max = airspd;
// //     //     // // Update flight submode
// //     //     //  if(airspd < airspd_min && pitch > threshold)
// //     //     //      isStall = true;
// //     // } else if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D && AP::ahrs().groundspeed()) {
// //     //     ahrs().airspeed_estimate(airspeed)
// //     //     // airspeed_EAS();
// //     //     airspeed_TAS();
// //     // }

// //     airspeed_TAS(taspd);
// //     if(AP::baro().healthy())
// //         alt = roundf(fabsf(AP::baro().get_altitude()));
// //         // if(alt_max < alt || (alt <= 1 && airspd <= 5))
// //         //     alt_max = alt;


// //     // // Limits: Height: >= 100 | 30 m, Airspeed >= 20 (Stall speed)
// //     // // Pitch > -60 deg;  Nose 30° Down: -30 degrees (or -0.52 radians)
// //     // // Detect limits violation including stalling  and swich to automatic recovery
// //     // if(Submode::Fbwa && airspeed)
// //     // AP::baro().healthy() && AP::baro().get_altitude() >= X
// //     // AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D && AP::ahrs().groundspeed() >= 3

// //     // if (plane.airspeed.enabled() && plane.airspeed.healthy()) {
// //     //     float airspeed_ms = plane.airspeed.get_airspeed();
// //     //     // Your flight mode logic here
// //     //      if(airspeed_ms < AS_<MIN && pitch > threshold)
// //     //          isStall = true;
// //     // }
// //     // AP::ahrs().airspeed_estimate(&estimated_airspeed)


// //     // Fetch the current pitch from AHRS (returned in radians)
// //     float current_pitch_rad = AP::ahrs().get_pitch();
// //     // Check if the nose is pointed 30 degrees down or lower
// //     if (current_pitch_rad <= DEG_TO_RAD * -30.0f) {
// //         // Your recovery or management logic here
// //     }

// //     // if(isStall) {
// //     //     pitch_target = negative small;  // If flight height allows
// //     //     throttle = max;
// //     //     roll_target = 0;
// //     // } else {
// //     //     // Stall exit condition
// //     //     isStall = false;
// //     //     restore_mode();
// //     // }

// //     switch(submode) {
// //     case Submode::Levelup:
// //         // isDirLocked = false;
// //         // Check exit conditions:
// //         if(alt && alt > alt_min + 5)
// //            submode =  Submode::Fbwa;

// //         // TODO: complete automaticcontrol to recover from the stall state (increase speed and then switch to the horizontal flight) and then adjust the target altitude and direction
// //         // plane.set_target_altitude_current();
// //         // target_yaw = AP::ahrs().get_yaw_rad();
// //         ///
// //         // submode =  Submode::Fbwa;
// //         break;
// //     case Submode::Headhold:
// //         // Lock in this
// //         headhold(isDirLocked, false);
// //         break;
// //     case Submode::Fbwa: 
// //     default: {
// //         // Fix directions by the RC Channel switch
// //         // Alternative: use plane.g2.dirlock_rcin
// //         chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::DIRLOCK);
// //         if (chan != nullptr && chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH) {
// //             headhold(isDirLocked, true);
// //             return;
// //         }
// //         isDirLocked = false;

// //         // set nav_roll and nav_pitch using sticks
// //         plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
// //         plane.update_load_factor();
// //         float pitch_input = plane.channel_pitch->norm_input();
// //         if (pitch_input > 0) {
// //             plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
// //         } else {
// //             plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
// //         }
// //         plane.adjust_nav_pitch_throttle();
// //         plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
// //         if (plane.fly_inverted()) {
// //             plane.nav_pitch_cd = -plane.nav_pitch_cd;
// //         }

// //         if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
// //             // FBWA failsafe glide
// //             plane.nav_roll_cd = 0;
// //             plane.nav_pitch_cd = 0;
// //             SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
// //         }
// //         chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
// //         if (chan != nullptr) {
// //             // check for the user enabling FBWA taildrag takeoff mode
// //             bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
// //             if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
// //                 if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
// //                     plane.auto_state.fbwa_tdrag_takeoff_mode = true;
// //                     plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
// //                 }
// //             }
// //         }

// //         // Check for the flight limits violations and switch to the respective submodes (stall recovery, height adjustment, direction lock)

// //     }
// // }

// // void ModeFBWT::run()
// // {
// //     // Run base class function and then output throttle
// //     Mode::run();

// //     output_pilot_throttle();
// // }
