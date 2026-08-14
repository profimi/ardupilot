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

    // ASCALAR(g_limit, "LIM_G_DEG", LIM_G_DEFAULT), // Linked directly under the aparm macro definitions
    
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

constexpr float _aoa_limit_deg = 12.0f;


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
float ModeFBWT::estimate_beta() const
{
    if (!ahrs.airspeed_sensor_enabled()) {
        return 0.0f; // fallback
    }
    Vector3f vel;
    if(!ahrs.get_velocity_NED(vel))
        vel.zero();

    float airspeed;
    ahrs.airspeed_EAS(airspeed);

    if (airspeed < 5.0f)
        return 0.0f;

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
// void ModeFBWT::apply_energy_roll_limit(float &roll_cmd)
// {
//     // AP_TECS *_tecs = plane._tecs;
//     if (!_tecs) return;

//     float STEdot = _tecs->get_VXdot(); 

//     if (STEdot < -3.0f) {
//         float scale = constrain_float((STEdot + 5.0f) / 2.0f, 0.3f, 1.0f);
//         roll_cmd *= scale;
//     }
// }

// Both airspeed, load factor, and Energy-aware (TECS) roll limiting
void ModeFBWT::apply_energy_roll_limit(float &roll_cmd, float airspeed)
{
    float roll_limit = plane.aparm.roll_limit;

    // --------------------------------------------------
    // 1. AIRSPEED-BASED LIMIT (PRIMARY SAFETY)
    // --------------------------------------------------
    float vmin = plane.aparm.airspeed_min;
    float vmax = plane.aparm.airspeed_max;

    float airspeed_factor = 1.0f;

    // Note: allow agressive stabilizing roll on takeoff
    // TODO: consider auto-detection of the takeoff phase (in both takeoff and other modes)
    // to allow aggressive stabilization specifically in the takeoff phase
    if (airspeed > 0.0f && plane.relative_altitude >= 5) {
        float t = constrain_float((airspeed - vmin) / (vmax - vmin), 0.0f, 1.0f);

        // Low speed → aggressive reduction
        airspeed_factor = 0.5f + 0.5f * t;
    } else {
        // No sensor → conservative fallback
        airspeed_factor = 0.6f;
    }

    float roll_lim_airspeed = roll_limit * airspeed_factor;

    // --------------------------------------------------
    // 2. LOAD FACTOR LIMIT (PHYSICS)
    // --------------------------------------------------
    float nz_limit = compute_adaptive_nz_limit(airspeed);

    float max_bank = degrees(acosf(1.0f / nz_limit));
    max_bank = constrain_float(max_bank, 20.0f, roll_limit);

    // --------------------------------------------------
    // 3. TECS ENERGY LIMIT (SECONDARY / SMOOTHING)
    // --------------------------------------------------
    float tecs_factor = 1.0f;

    // float energy_error = _tecs->get_STE_error();
    // Calculate altitude error (Potential Energy component)
    // float alt_error = _tecs->get_target_altitude() - plane.current_loc.alt * 0.01f;
    // float alt_error = (plane.nav_altitude_cm * 0.01f) - plane.current_loc.alt * 0.01f;
    // Extract the master AMSL target height from the target_altitude tracker block
    float alt_error = (plane.target_altitude.amsl_cm * 0.01f) - (plane.current_loc.alt * 0.01f);

    // Calculate airspeed error (Kinetic Energy component)
    float airspeed_error = _tecs->get_target_airspeed() - plane.airspeed.get_airspeed();

    // Combine them into a Total Specific Energy Error (Standard Total Energy Error formulation)
    // Total Energy = Potential Energy (height * gravity) + Kinetic Energy (0.5 * velocity^2)
    float energy_error = (alt_error * 9.81f) + (0.5f * (airspeed_error * airspeed_error));

    if (energy_error < 0.0f) {
        tecs_factor = constrain_float(1.0f + energy_error * 0.3f, 0.5f, 1.0f);
    }

    float roll_lim_tecs = roll_limit * tecs_factor;

    // --------------------------------------------------
    // 4. FINAL LIMIT = MOST CONSERVATIVE
    // --------------------------------------------------
    float final_limit = MIN(roll_lim_airspeed,
                      MIN(max_bank, roll_lim_tecs));

    roll_cmd = constrain_symmetric(roll_cmd, final_limit);
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

    apply_flare_protection(pitch_cmd);
}

// Near-ground pitch angle control protection
void ModeFBWT::apply_flare_protection(float &pitch_cmd)
{
    float height = plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING);  // Or use RangeFinderUse::NONE

    const float flare_start = 10.0f; // meters
    const float flare_end   = 2.0f;

    if (height > flare_start)
        return;

    float t = constrain_float(
        (height - flare_end) / (flare_start - flare_end),
        0.0f,
        1.0f
    );

    // limit pitch-up aggressively near ground
    float max_pitch = plane.aparm.pitch_limit_max * t;

    if (pitch_cmd > max_pitch)
        pitch_cmd = max_pitch;

    // optional: slight nose-down bias for safety
    if (height < 5.0f)
        pitch_cmd = MIN(pitch_cmd, 5.0f);
}

// TECS energy limiter
void ModeFBWT::apply_energy_limiter(float &pitch_cmd)
{
    // AP_TECS *_tecs = plane._tecs;
    if (!_tecs)
        return;

    float STEdot = _tecs->get_VXdot();

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
    // If rudder is pilot-controlled, don't override aggressively
    float rudder_in = plane.channel_rudder->norm_input();

    // Get airspeed (fallback-safe)
    float airspeed = 0;
    ahrs.airspeed_EAS(airspeed);
    if (airspeed < 5.0f)
        return;

    // Convert roll command to radians
    float phi = radians(roll_cmd);

    // Coordinated turn yaw rate (rad/s)
    const float g = GRAVITY_MSS;
    float yaw_rate_target = g * tanf(phi) / airspeed;

    // Measured yaw rate
    float yaw_rate_meas = plane.ahrs.get_gyro().z;

    // Yaw rate error
    float yaw_error = yaw_rate_target - yaw_rate_meas;

    // Beta correction (sideslip damping)
    float beta = estimate_beta();
    float beta_gain = 0.5f;
    float beta_correction = -beta_gain * beta;

    // Combine
    float rudder_cmd = yaw_error * 0.8f + beta_correction;

    // Blend with pilot input (do not fight pilot)
    float blend = 0.7f;
    float final_rudder = blend * rudder_cmd + (1.0f - blend) * rudder_in;

    // Send to controller
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, final_rudder * 4500);  // centidegrees scale
}

// Envelope Style factor (trainer .. aggressive)
float ModeFBWT::get_envelope_style()
{
    // Use existing tuning to infer "aggressiveness"

    float roll_max = plane.aparm.roll_limit; // deg
    float pitch_max = plane.aparm.pitch_limit_max;

    // Normalize (trainer → aggressive)
    float roll_factor  = constrain_float(roll_max / 60.0f, 0.5f, 1.5f);
    float pitch_factor = constrain_float(pitch_max / 30.0f, 0.5f, 1.5f);

    float style = 0.5f * (roll_factor + pitch_factor);

    // Clamp final style
    return constrain_float(style, 0.5f, 1.5f);
}

// Apply envelope shaping
void ModeFBWT::apply_envelope_shaping(float &roll_lim, float &pitch_lim)
{
    float style = get_envelope_style();

    // Trainer → softer, Aggressive → sharper
    float trainer_blend = 1.0f / style;

    // Roll shaping
    roll_lim *= trainer_blend;

    // Pitch shaping
    pitch_lim *= trainer_blend;

    // Add slight damping for trainer mode
    if (style < 1.0f) {
        roll_lim  *= 0.9f;
        pitch_lim *= 0.9f;
    }
}

// Levelup trigger
bool ModeFBWT::is_below_alt_min() const
{
    int16_t alt = roundf(plane.relative_altitude);  // meters
    float vel_down;
    float sink_rate = 0;
    const AP_GPS &gps = AP::gps();
    if (ahrs.get_velocity_D(vel_down))
        sink_rate = vel_down;
    else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity())
        sink_rate = gps.velocity().z;
    else sink_rate = -plane.barometer.get_climb_rate();
    // // Smooth the noise
    // auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f * sink_rate;    

    return (sink_rate > 0 && alt < alt_min + sink_rate*3);
}

// Levelup controller
void ModeFBWT::apply_levelup_protection(float &roll_cmd, float &pitch_cmd)
{
    // Wings level aggressively
    roll_cmd = blend_limits(roll_cmd, 0.0f, 0.2f);

    // Force safe pitch-up (but not extreme)
    float pitch_target = MIN(10.0f, plane.aparm.pitch_limit_max);

    pitch_cmd = blend_limits(pitch_cmd, pitch_target, 0.1f);

    // Prevent nose-down
    pitch_cmd = MAX(pitch_cmd, 5.0f);
    _submode = Submode::Levelup;
}

void ModeFBWT::apply_final_envelope(float &roll_cmd, float &pitch_cmd)
{
    float roll_lim  = plane.aparm.roll_limit;
    float pitch_lim = plane.aparm.pitch_limit_max;

    // 1) Envelope shaping (trainer/aggressive)
    apply_envelope_shaping(roll_lim, pitch_lim);

    // 2) Apply limits
    roll_cmd  = constrain_symmetric(roll_cmd, roll_lim);
    pitch_cmd = constrain_symmetric(pitch_cmd, pitch_lim);

    // 3) Low-altitude protection (highest priority)
    if (is_below_alt_min())
        apply_levelup_protection(roll_cmd, pitch_cmd);
    else _submode = Submode::Fbwa;
}

// Angle of Attack estimation and protection
float ModeFBWT::estimate_aoa() const
{
    float flight_path = 0;
    {
        float vel_down;
        if(ahrs.get_velocity_D(vel_down)) {
            float airspeed;
            ahrs.airspeed_EAS(airspeed);
            flight_path = degrees(atan2f(-vel_down, airspeed));
        }
    }
    float pitch = ahrs.get_pitch_deg();

    return pitch - flight_path;
}

float ModeFBWT::get_fused_aoa() const
{
    float aoa_est = estimate_aoa();

    if (!plane.airspeed.use()) {
        return aoa_est;
    }

    float aoa_meas = plane.ahrs.getAOA(); 

    // confidence weighting
    float w = 0.7f; // trust sensor more near stall

    // optional: reduce trust at high noise / low speed
    float airspeed;
    if (ahrs.airspeed_EAS(airspeed) && airspeed < plane.aparm.airspeed_min * 1.2f) {
        w = 0.9f;
    }

    return w * aoa_meas + (1.0f - w) * aoa_est;
}

void ModeFBWT::apply_aoa_protection(float &pitch_cmd, float aoa)
{
    const float aoa_limit = _aoa_limit_deg; // conservative

    if (aoa > aoa_limit && pitch_cmd > 0) {
        float scale = aoa_limit / aoa;
        pitch_cmd *= scale;
    }
}

void ModeFBWT::apply_aoa_rate_damping(float &pitch_cmd)
{
    static float prev_aoa = 0.0f;

    float aoa = get_fused_aoa();
    float aoa_rate = (aoa - prev_aoa) / MAX(plane.G_Dt, 0.01f);

    prev_aoa = aoa;

    const float rate_limit = 20.0f; // deg/sec

    if (aoa_rate > rate_limit && pitch_cmd > 0) {
        float scale = rate_limit / aoa_rate;
        pitch_cmd *= scale;
    }
}

void ModeFBWT::apply_aoa_limiter(float &pitch_cmd)
{
    float aoa_meas = 0.0f;
    float aoa_est  = estimate_aoa();   // fallback estimate
    bool aoa_valid = false;

#if AP_AHRS_ENABLED
    if (plane.ahrs.airspeed_sensor_enabled()) {
        aoa_meas = plane.ahrs.getAOA();
        aoa_valid = isfinite(aoa_meas);
    }
#endif

    // Blend measured + estimated AoA
    float aoa = aoa_valid ? (0.7f * aoa_meas + 0.3f * aoa_est) : aoa_est;

    const float aoa_soft = radians(12.0f);
    const float aoa_hard = radians(15.0f);

    if (aoa > aoa_soft) {

        float t = constrain_float((aoa - aoa_soft) / (aoa_hard - aoa_soft), 0.0f, 1.0f);

        // progressively remove pitch-up authority
        float scale = 1.0f - t;

        if (pitch_cmd > 0) {
            pitch_cmd *= scale;
        }

        // hard clamp at stall
        if (aoa > aoa_hard) {
            pitch_cmd = MIN(pitch_cmd, 0.0f);
        }
    }
}

// Combined envelope limiter
void ModeFBWT::apply_envelope_limits(float &roll_cmd, float &pitch_cmd)
{
    float airspeed;
    ahrs.airspeed_EAS(airspeed);
    float Nz = estimate_load_factor(roll_cmd);

    // Load protection
    apply_nz_limit(roll_cmd, airspeed);
    apply_energy_roll_limit(roll_cmd, airspeed);

    // Coordinated flight
    apply_turn_coordination(roll_cmd);

    // Stall protection
    apply_stall_protection(pitch_cmd, airspeed, Nz);
    // AoA-based (if sensor or estimate available)
    float aoa = get_fused_aoa();
    apply_aoa_protection(pitch_cmd, aoa);  // 1. Soft AoA-based shaping
    // Dynamic CL-based stall protection
    apply_dynamic_stall(pitch_cmd, Nz);
    // AoA-rate damping (pre-buffer)
    apply_aoa_rate_damping(pitch_cmd);  // 2. Dynamic AoA-based dumping
    apply_aoa_limiter(pitch_cmd);  // 3. Hard AoA-baed limit (last)

    // TECS-based energy management
    apply_energy_limiter(pitch_cmd);
    // Terrain / low altitude protection
    apply_terrain_protection(pitch_cmd);

    apply_final_envelope(roll_cmd, pitch_cmd);

    EnvelopeState est;
    update_envelope_state(est);
    log_envelope(est);
}

void ModeFBWT::update_envelope_state(EnvelopeState &env)
{
    // --- AoA ---
    env.aoa = degrees(plane.ahrs.getAOA());
    env.aoa_limit = 12;  // aoa_protection_limit_deg;  // TODO: Might define the respective parameter

    env.aoa_margin = constrain_float(
        (env.aoa_limit - env.aoa) / env.aoa_limit,
        0.0f, 1.0f
    );

    // --- G-load ---
    // Fetches the real-time G-force vector along the aircraft's body axes
    Vector3f accel_eff = plane.ahrs.get_accel_ef();
    // The z-component contains the vertical G-loading (divided by gravity)
    // Note: Ardupilot conventions mean this is negative under positive Gs, 
    // so take the absolute value or negate it depending on your context.
    env.load_factor = fabsf(accel_eff.z) / GRAVITY_MSS;
    env.g_limit = 2.5f;  // plane.aparm.g_limit;;  // g_limit_max;  // TODO: Might define the respective parameter

    env.g_margin = constrain_float(
        (env.g_limit - env.load_factor) / env.g_limit,
        0.0f, 1.0f
    );

    // --- Airspeed ---
    if (plane.airspeed.use()) {
        // Synthetic Fallback path: Use when sensor is absent, disabled, or failed
        // This extracts the EKF's groundspeed-minus-wind mathematical estimation
        if (!plane.ahrs.airspeed_EAS(env.airspeed)) {
            // Absolute worst-case scenario backup if EKF estimate is uninitialized
            env.airspeed = plane.ahrs.groundspeed();
        }
    } else env.airspeed = plane.airspeed.get_airspeed();
    env.v_min = plane.aparm.airspeed_min;  // m/s

    env.v_margin = constrain_float(
        (env.airspeed - env.v_min) / env.v_min,
        0.0f, 1.0f
    );

    // --- Energy (TECS) ---
    // 1. Calculate Actual Specific Total Energy (STE = SPE + SKE)
    // SPE (Potential) = Height * Gravity
    // SKE (Kinetic) = 0.5 * True Airspeed squared
    float actual_height = plane.current_loc.alt * 0.01f;
    // True Airspeed calculation fallback
    float actual_tas = 0.0f;
    if (plane.airspeed.use()) {
        // Use the physical pitot sensor converted to True Airspeed
        actual_tas = plane.airspeed.get_airspeed() * plane.ahrs.get_EAS2TAS();
    } else if (!plane.ahrs.airspeed_TAS(actual_tas)) {
        // Fallback if the sensor is disabled and EKF wind tracking is uninitialized
        actual_tas = plane.ahrs.groundspeed(); 
    }
    float e_total = (actual_height * GRAVITY_MSS) + (0.5f * actual_tas * actual_tas);

    // 2. Calculate Target Specific Total Energy Demand
    float target_height = plane.target_altitude.amsl_cm * 0.01f;
    float target_tas = _tecs->get_target_airspeed() * plane.ahrs.get_EAS2TAS(); // Convert target EAS to TAS
    float e_target = (target_height * GRAVITY_MSS) + (0.5f * target_tas * target_tas);

    env.energy_error = e_target - e_total;
    // Calculate an aircraft-specific dynamic maximum variance bounds
    float min_safe_speed = plane.aparm.airspeed_min;
    float max_maneuver_g = env.g_limit;  // plane.aparm.g_limit;

    // Combine worst-case speed dropping to zero + high-G maneuver energy shedding estimation
    float energy_error_max = (0.5f * min_safe_speed * min_safe_speed) + (max_maneuver_g * GRAVITY_MSS * 10.0f);

    // Ensure it never causes a divide-by-zero compile trap or run-time freeze
    if (energy_error_max < 1.0f) {
        energy_error_max = 200.0f;
    }

    env.energy_margin = constrain_float(
        1.0f - fabsf(env.energy_error) / energy_error_max,
        0.0f, 1.0f
    );

    // --- Limiter flags ---
    env.limiter_flags = 0;

    if (env.aoa_margin < 0.2f) env.limiter_flags |= 1 << 0;
    if (env.g_margin   < 0.2f) env.limiter_flags |= 1 << 1;
    if (env.v_margin   < 0.2f) env.limiter_flags |= 1 << 2;
    if (env.energy_margin < 0.2f) env.limiter_flags |= 1 << 3;
}

// Main function
void ModeFBWT::update()
{
    float pitch_in = plane.channel_pitch->norm_input();  // -1..1
    float roll_in  = plane.channel_roll->norm_input();
    // Stick shaping (more natural feel)
    if(ctl_expocrv > 0) {
        pitch_in = expo_curve(pitch_in, ctl_expocrv);
        roll_in = expo_curve(pitch_in, ctl_expocrv);
    }

    float pitch_max = plane.aparm.pitch_limit_max;
    float pitch_min = plane.aparm.pitch_limit_min;
    float roll_max  = plane.aparm.roll_limit;

    float pitch_cmd = pitch_in * (pitch_in > 0 ? pitch_max : -pitch_min);
    float roll_cmd  = roll_in  * roll_max;

    switch (_submode) {
    case Submode::Levelup:
        if (pitch_in > 0.8f) {
            pitch_cmd = pitch_max;
            roll_cmd = blend_limits(roll_cmd, 0.0f, 0.2f);
        }
        break;
    // case Submode::Headhold:
    //     if (fabsf(roll_in) < 0.05f) {
    //         roll_cmd = plane.nav_roll_cd * 0.01f;
    //     }
    //     break;
    case Submode::Fbwa:
    default:
        break;
    }

    apply_envelope_limits(roll_cmd, pitch_cmd);

    plane.nav_pitch_cd = pitch_cmd * 100.0f;
    plane.nav_roll_cd  = roll_cmd  * 100.0f;

    if (plane.g.log_bitmask & MASK_LOG_ATTITUDE_FAST) {
        float airspeed;
        ahrs.airspeed_EAS(airspeed);
        AP::logger().Write(
            "FBWT",
            "TimeUS,Rin,Pin,Rcmd,Pcmd,Nz,Spd,Beta",
            "Qfffffff",
            AP_HAL::micros64(),
            roll_in,
            pitch_in,
            roll_cmd,
            pitch_cmd,
            estimate_load_factor(roll_cmd),
            airspeed,
            estimate_beta()
        );
    }
}

void ModeFBWT::log_envelope(const EnvelopeState &env) const
{
    AP::logger().Write("ENVP",
        "TimeUS,AoA,AoALim,AoAMargin,G,GLim,GMargin,V,Vmin,VMargin,Eerr,Emargin,Flags",
        "QffffffffffffB",
        AP_HAL::micros64(),
        env.aoa,
        env.aoa_limit,
        env.aoa_margin,
        env.load_factor,
        env.g_limit,
        env.g_margin,
        env.airspeed,
        env.v_min,
        env.v_margin,
        env.energy_error,
        env.energy_margin,
        env.limiter_flags
    );
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
//     const float pitch_max = plane.aparm.pitch_limit_max;
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
//     const float pitch_max = plane.aparm.pitch_limit_max;
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
//     float pitch = ahrs.get_pitch_rad();
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

// //     switch (_submode) {

// //     case Submode::Fbwa:
// //         if (stall || low_alt) {
// //             _submode = Submode::Levelup;
// //             assist_gain = 1.0f;
// //         }
// //         break;

// //     case Submode::Levelup:
// //         if (alt > alt_min + alt_hyst) {
// //             _submode = Submode::Headhold;
// //             target_alt = alt;
// //             target_heading = plane.ahrs.yaw;
// //         }
// //         break;

// //     case Submode::Headhold:
// //         assist_gain *= assist_decay;
// //         if (assist_gain < 0.2f) {
// //             _submode = Submode::Fbwa;
// //         }
// //         break;
// //     }
// // }

// // float ModeFBWT::compute_alpha()
// // {
// //     return get_aoa();
// //     // fallback estimate
// //     float vz = 0;
// //     if(plane.ahrs.have_velocity()) {
// //         float vel_down;
// //         ahrs.get_velocity_D(vel_down);
// //         vz = -vel_down;
// //     }
// //     return ahrs.get_pitch_rad() - atan2f(vz, cur_airspeed);
// // }

// // float ModeFBWT::get_aoa() const
// // {
// //     // Fallback: synthetic AoA approximation
// //     float pitch = ahrs.get_pitch_deg();
// //     float flight_path_angle = get_flight_path_angle();

// //     return pitch - flight_path_angle;
// // }

// // float ModeFBWT::get_flight_path_angle() const
// // {
// //     if (plane.ahrs.have_velocity()) {
// //         Vector3f vel;
// //         if(!ahrs.get_velocity_NED(vel))
// //           return 0;
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
//     return plane.ahrs.get_location(loc);
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
//     switch(_submode) {
//     case Submode::Levelup:  // Climb
//         return 0.8f;   // favor rate
//     case Submode::Headhold: 
//         return 0.2f;   // favor absolute
//     case Submode::Fbwa:   // Maneuvering
//         return 0.9f;   // almost all rate
//     }
//     return 0.5f; // fallback
// }

// // // float ModeFBWT::energy_rate()
// // // {
// // //     float vz = 0;
// // //     if(plane.ahrs.have_velocity()) {
// // //        float vel_down;
// // //        ahrs.get_velocity_D(vel_down);
// // //        vz = -vel_down;
// // //     }
// // //     float v  = cur_airspeed;
// // //
// // //     return vz + (v * plane.airspeed_rate) / GRAVITY_MSS;
// // // }

// // float ModeFBWT::energy_rate()
// // {
// //     // --- 1. Vertical speed (positive up)
// //     float vz = 0.0f;
// //     if (has_velocity) {
// //         float vel_down;
// //         ahrs.get_velocity_D(vel_down);
// //         vz = -vel_down;
// //     }

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

float ModeFBWT::energy_rate() const
{
    // --- 1. Vertical speed (m/s, positive up)
    float vz = 0.0f;

    float vel_down;
    if (plane.ahrs.get_velocity_D(vel_down)) {
        vz = -vel_down;  // Negate to get positive vertical climb rate (vz)
    }

    // --- 2. Speed estimate (m/s)
    float v = plane.airspeed.get_airspeed();  // cur_airspeed;

    // --- 3. Forward acceleration (body X axis)
    float ax = 0.0f;

    const AP_InertialSensor &ins = AP::ins();
    if (ins.get_accel_count() > 0)
        ax = ins.get_accel(0).x;  // body-frame forward accel

    // --- 4. Energy rate
    // return vz + (v * ax) / GRAVITY_MSS;

    static float ax_filt = 0.0f;
    const float alpha = 0.2f;

    ax_filt = alpha * ax + (1.0f - alpha) * ax_filt;

    return vz + (v * ax_filt) / GRAVITY_MSS;
}

// float ModeFBWT::energy_rate() {
//     float ax = ahrs.get_accel_body().x;
//     return vz + (v * ax) / GRAVITY_MSS;
// }

// // float ModeFBWT::energy_rate()
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
// //         plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit * 100;
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
//     _tecs->update_pitch_throttle(
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
//     switch (_submode) {
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

// // ModeFBWT::ModeFBWT(): isDirLocked{false}, _submode{Submode::Fbwa}, alt_max{0}, airspd_max{0}
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
// //     plane.nav_roll_cd = constrain_int32(plane.nav_controller->nav_roll_cd(), -plane.roll_limit*100, plane.roll_limit*100);
// //     plane.nav_pitch_cd = locked_pitch_cd;

// //     // Note: Throttle locking is performed in Plane::set_throttle(void), otherwise the value is set there anyway overwriting the current one
// //     // // Set fixed throttle
// //     // SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, locked_throttle);
// // }

bool ModeFBWT::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    if (!AP::ahrs().healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: AHRS is not healthy, FBWT is unstable");
        // Note: It makes sense to allow FBWC even for unhealthy AHRS, moving the responsibility to pilot
        // return false;
    }

    // plane.set_target_altitude_current();
    // target_yaw = AP::ahrs().get_yaw_rad();

    // Use ArduPlane's existing TECS controller.
    _tecs = &plane.TECS_controller;

    // Resent the submode toe the FBWA like
    _submode = Submode::Fbwa;
    // isDirLocked = false;

    // // Reset FBWT state.
    // _aoa_fused = 0.0f;
    // _aoa_rate = 0.0f;
    // _prev_aoa = 0.0f;
    //
    // _pitch_cmd_prev = 0.0f;
    // _roll_cmd_prev = 0.0f;
    //
    // _active_limiters = 0;

    return true;
}

void ModeFBWT::_exit()
{
    _tecs = nullptr;

    // _active_limiters = 0;
}

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
// //     //     ahrs().airspeed_EAS(airspeed)
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
// //     // AP::ahrs().airspeed_EAS(&estimated_airspeed)


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

// //     switch(_submode) {
// //     case Submode::Levelup:
// //         // isDirLocked = false;
// //         // Check exit conditions:
// //         if(alt && alt > alt_min + 5)
// //            _submode =  Submode::Fbwa;

// //         // TODO: complete automaticcontrol to recover from the stall state (increase speed and then switch to the horizontal flight) and then adjust the target altitude and direction
// //         // plane.set_target_altitude_current();
// //         // target_yaw = AP::ahrs().get_yaw_rad();
// //         ///
// //         // _submode =  Submode::Fbwa;
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
// //         plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit * 100;
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

// float ModeFBWT::get_recovery_assist_gain() const
// {
//     if (_submode == Submode::Fbwa)  // _submode != Submode::Levelup
//         return 0.0f;
// 
//     float aoa = get_fused_aoa();
//     float energy = energy_factor();
// 
//     float aoa_need =
//         constrain_float(
//             (aoa - aoa_soft_limit) /
//             (aoa_hard_limit - aoa_soft_limit),
//             0.0f, 1.0f);
// 
//     float energy_need = 1.0f - energy;
// 
//     float assist = MAX(aoa_need, energy_need);
// 
//     return constrain_float(assist, 0.0f, 1.0f);
// }
// 
// void ModeFBWT::output_fbwt_throttle_assist()
// {
//     float pilot = plane.channel_throttle->norm_input();
// 
//     // 0..1, where 1 means strong recovery assistance.
//     float assist = get_recovery_assist_gain();
// 
//     // Automatic recovery throttle demand.
//     float recovery = compute_recovery_throttle();
// 
//     float throttle =
//         pilot * (1.0f - assist) +
//         recovery * assist;
// 
//     throttle = constrain_float(throttle, 0.0f, 1.0f);
// 
//     plane.throttle_suppression = false;
//     plane.set_servos_manual_passthrough(); // NOT necessarily appropriate
// }

// // Fast, Energy rate, noisy
// float ModeFBWT::energy_factor() const
// {
//     float e_rate = energy_rate();
//
//     if (e_rate > -0.5f) return 1.0f;
//
//     return constrain_float(1.0f + e_rate, 0.3f, 1.0f);
// }

float ModeFBWT::energy_factor() const
{
    const float e_rate = energy_rate();

    if (!isfinite(e_rate)) {
        return 1.0f;
    }

    /*
     * Positive or approximately neutral energy rate:
     * no throttle assistance required.
     */
    if (e_rate >= -0.5f) {
        return 1.0f;
    }

    /*
     * -1.0 -> 0.0
     * -0.5 -> 0.5
     *
     * Clamp the lower bound so the protection doesn't
     * disappear completely.
     */
    return constrain_float(
        1.0f + e_rate,
        0.3f,
        1.0f
    );
}

float ModeFBWT::compute_recovery_throttle() const
{
    /*
     * Start at the aircraft's normal cruise throttle and
     * increase toward THR_MAX according to energy deficit.
     */

    float cruise =
        constrain_float(
            plane.aparm.throttle_cruise * 0.01f,
            0.0f,
            1.0f
        );

    float maximum =
        constrain_float(
            plane.aparm.throttle_max * 0.01f,
            0.0f,
            1.0f
        );


    if (maximum < cruise) {
        maximum = cruise;
    }


    // ------------------------------------------------------------
    // Energy state
    // ------------------------------------------------------------

    const float energy_factor =
        constrain_float(
            ModeFBWT::energy_factor(),
            0.0f,
            1.0f
        );

    const float deficit =
        1.0f - energy_factor;


    // ------------------------------------------------------------
    // Cruise -> maximum throttle.
    // ------------------------------------------------------------

    float recovery =
        cruise +
        deficit * (maximum - cruise);


    // ------------------------------------------------------------
    // Severe stall condition:
    //
    // Don't reduce throttle below cruise, but don't use AoA
    // itself as a reason to blindly command maximum throttle.
    // The AoA limiter is responsible for unloading the aircraft.
    // ------------------------------------------------------------

    const float aoa = get_fused_aoa();

    if (isfinite(aoa) && aoa > _aoa_limit_deg) {
        recovery = MAX(recovery, cruise);
    }


    return constrain_float(
        recovery,
        cruise,
        maximum
    );
}

float ModeFBWT::get_throttle_assist_gain() const
{
    if (_submode != Submode::Levelup)
        return 0.0f;

    // ------------------------------------------------------------
    // Energy deficit
    // ------------------------------------------------------------

    const float energy_factor =
        constrain_float(
            ModeFBWT::energy_factor(),
            0.0f,
            1.0f
        );

    const float energy_deficit =
        1.0f - energy_factor;


    // ------------------------------------------------------------
    // AoA severity
    // ------------------------------------------------------------

    float aoa_gain = 0.0f;

    const float aoa = get_fused_aoa();
    constexpr float _aoa_soft_zone_deg = 3.f;

    if (isfinite(aoa) && _aoa_limit_deg > _aoa_soft_zone_deg) {
        aoa_gain =
            constrain_float(
                (aoa - _aoa_soft_zone_deg) /
                (_aoa_limit_deg - _aoa_soft_zone_deg),
                0.0f,
                1.0f
            );
    }


    // ------------------------------------------------------------
    // Recovery demand
    //
    // Energy is the main reason for throttle assistance.
    // AoA can increase assistance when the aircraft is close
    // to the stall-protection boundary.
    // ------------------------------------------------------------

    float demand =
        MAX(
            energy_deficit,
            aoa_gain * 0.5f
        );


    // ------------------------------------------------------------
    // Don't immediately jump to 100% throttle.
    //
    // This is intentionally bounded without introducing another
    // FBWT parameter.
    // ------------------------------------------------------------

    demand =
        constrain_float(
            demand,
            0.0f,
            1.0f
        );

    return demand;
}

void ModeFBWT::output_fbwt_throttle_assist()
{
    /*
     * FBWT LEVELUP throttle assistance.
     *
     * Pilot throttle remains the baseline command.
     * FBWT only adds bounded assistance when recovery
     * requires additional energy.
     *
     * We deliberately preserve the two paths used by
     * Mode::output_pilot_throttle():
     *
     *   THR_PASS_STAB:
     *       plane.get_throttle_input(true)
     *
     *   normal FBWA-style throttle:
     *       plane.get_adjusted_throttle_input(true)
     */

    float pilot_throttle;

    // ------------------------------------------------------------
    // 1. Obtain pilot throttle using the same mechanism as
    //    Mode::output_pilot_throttle().
    // ------------------------------------------------------------

    if (plane.g.throttle_passthru_stabilize) {

        pilot_throttle =
            plane.get_throttle_input(true);

    } else {

        pilot_throttle =
            plane.get_adjusted_throttle_input(true);
    }


    // ------------------------------------------------------------
    // 2. Convert the result to normalized 0..1.
    //
    // ArduPlane's throttle-input helpers return the scaled
    // throttle representation used by SRV_Channels.
    // ------------------------------------------------------------

    pilot_throttle =
        constrain_float(
            pilot_throttle * 0.01f,
            0.0f,
            1.0f
        );


    // ------------------------------------------------------------
    // 3. Determine how much automatic assistance is required.
    // ------------------------------------------------------------

    const float assist_gain =
        get_throttle_assist_gain();


    // ------------------------------------------------------------
    // 4. No assistance -> exactly the normal pilot path.
    // ------------------------------------------------------------

    if (assist_gain <= 0.001f) {

        if (plane.g.throttle_passthru_stabilize) {

            SRV_Channels::set_output_scaled(
                SRV_Channel::k_throttle,
                plane.get_throttle_input(true)
            );

        } else {

            SRV_Channels::set_output_scaled(
                SRV_Channel::k_throttle,
                plane.get_adjusted_throttle_input(true)
            );
        }

        return;
    }


    // ------------------------------------------------------------
    // 5. Calculate the recovery throttle.
    // ------------------------------------------------------------

    const float recovery_throttle =
        compute_recovery_throttle();


    // ------------------------------------------------------------
    // 6. Blend pilot and recovery commands.
    //
    // assist_gain:
    //
    //     0 -> 100% pilot
    //     1 -> 100% recovery command
    //
    // Normally LEVELUP remains somewhere between these.
    // ------------------------------------------------------------

    float throttle =
        pilot_throttle * (1.0f - assist_gain) +
        recovery_throttle * assist_gain;


    // ------------------------------------------------------------
    // 7. Respect existing ArduPlane throttle limits.
    //
    // These are standard parameters:
    //
    //   THR_MIN
    //   THR_MAX
    //
    // See ArduPlane Parameters.cpp.
    // ------------------------------------------------------------

    const float throttle_min =
        constrain_float(
            plane.aparm.throttle_min * 0.01f,
            0.0f,
            1.0f
        );

    const float throttle_max =
        constrain_float(
            plane.aparm.throttle_max * 0.01f,
            0.0f,
            1.0f
        );

    throttle =
        constrain_float(
            throttle,
            throttle_min,
            throttle_max
        );


    // ------------------------------------------------------------
    // 8. Output using the same ArduPlane output interface as
    //    Mode::output_pilot_throttle().
    // ------------------------------------------------------------

    SRV_Channels::set_output_scaled(
        SRV_Channel::k_throttle,
        throttle * 100.0f
    );
}

void ModeFBWT::run()
{
    // Common fixed-wing attitude/stick-mixing processing
    Mode::run();

    if (_submode != Submode::Fbwa)  // _submode == Submode::Levelup
        output_fbwt_throttle_assist();
    else output_pilot_throttle();
}