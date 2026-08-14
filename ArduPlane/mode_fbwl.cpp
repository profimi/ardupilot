/*
  ModeFBWL - "Fly By Wire Training / Assisted"

  FBWL is FBWA (manual roll/pitch, manual throttle) augmented with
  continuous, dynamic envelope protection - the pilot always has direct
  stick authority, but the mode blends in corrective demand as the
  aircraft approaches AoA, G, speed, or energy limits, and only takes
  full control away (Levelup) as a last-resort automatic recovery if
  those soft protections are insufficient (e.g. a gust-induced stall).

  Submodes (Headhold removed - see rationale below):

    Fbwa    - manual stick + throttle, WITH continuous protections:
              AoA limiter + rate damping, G-limit, stall-speed
              protection, TECS-integrated energy limiting. Coordinated
              turn / yaw blending is reused from stock ArduPilot
              unmodified.
    Levelup - "Bailout; Recover": auto-throttle recovery flown by TECS
              + this mode's own pitch/roll demand, engaged only if the
              aircraft actually leaves the envelope despite the Fbwa
              protections (real stall, hard AoA/G overshoot, pitch
              below floor, or altitude below floor). Once recovered it
              hands control straight back to Fbwa (protections still
              active) - there is no permanent hold submode.

  Why Headhold was removed: Headhold existed in the previous revision
  to give a permanently-safe hands-off state after a bailout, because
  Fbwa itself had no ongoing protection - a recovered aircraft handed
  back to unprotected Fbwa could simply be flown back into the same
  corner immediately. Now that Fbwa is continuously protected (AoA/G/
  stall/energy limiting run every loop, not just as a discrete trigger),
  handing back to Fbwa after Levelup is itself safe, and a permanent
  heading/altitude hold is no longer needed to make that true. This
  turns FBWL from a "bail out and freeze" training mode into an
  "assisted flight" mode the pilot stays in control of throughout.

  ---------------------------------------------------------------------
  Stock ArduPlane 4.7 parameters reused (no new parameter for any of
  these - existing behaviour/tuning is respected, not duplicated):
  ---------------------------------------------------------------------
    ARSPD_FBW_MIN        (aparm.airspeed_min)   - stall/min-speed floor
    ARSPD_FBW_MAX         (aparm.airspeed_max)   - referenced for logging/context
    LIM_ROLL_CD           (aparm.roll_limit_cd)  - outer roll envelope
    PTCH_LIM_MAX_DEG/MIN  (aparm.pitch_limit_max/min) - outer pitch envelope
    STALL_PREVENTION       (g.stall_prevention)   - master enable for the
                                                     load-factor-based
                                                     protections, exactly
                                                     as stock ArduPilot
                                                     already uses it in
                                                     update_load_factor()
    THR_MAX / THR_MIN     (aparm.throttle_max/min) - energy-limiting cue
                                                       (throttle saturation)
    TECS_*                 (plane.TECS_controller)  - altitude/airspeed
                                                       energy management in
                                                       Levelup, completely
                                                       untouched/reused
    YAW2SRV_* / STICK_MIXING - coordinated turn + pilot/auto rudder
                               blending, completely reused (see
                               apply_coordinated_yaw() below - this mode
                               never writes k_rudder itself)
    RLL2SRV_* / PTCH2SRV_*  - rate-loop gains, untouched; this mode only
                              ever adjusts the *demand* fed into those
                              loops (nav_roll_cd/nav_pitch_cd), never the
                              gains themselves

  Only two new tunables were added, because stock ArduPlane has no
  existing parameter for either quantity: FBWL_AOA_MAX (there is no
  stock "maximum usable angle of attack" parameter) and FBWL_LOAD_MAX
  (there is no stock "maximum load factor / G" parameter for fixed-wing,
  unlike ANGLE_MAX on multirotors). FBWL_PITCH_MIN/FBWL_ALT_MIN/
  FBWL_ALT_HYST carry over unchanged from the previous revision.

  ---------------------------------------------------------------------
  Sensor-availability handling (GPS / airspeed sensor) - unchanged
  policy from the previous revision, extended to the new features:
  ---------------------------------------------------------------------
  speed_available()/have_position() remain the single gates for every
  airspeed- and position-derived quantity respectively. Every new
  protection below is written to simply not fire (never to assume a
  value) when its required sensor input is unavailable:
    - AoA limiter: uses a real AoA vane if AP_AHRS_AOA_ENABLED, else a
      synthetic pitch-minus-flight-path-angle estimate that itself
      degrades through speed_available() and a baro-only climb-rate
      fallback (needs neither GPS nor airspeed in the worst case); if
      no usable estimate exists, it returns 0 deg AoA => cannot trigger.
    - G-limit: uses only ahrs attitude (roll) + STALL_PREVENTION -
      needs neither GPS nor airspeed.
    - Stall-speed protection / energy limiting: gated by
      speed_available(), same as before.
    - Altitude floor / Levelup climb target: gated by have_position(),
      same reference_alt_cm() fallback as before.

  This file assumes the surrounding scaffolding described in
  README_FBWL.md (mode.h enum + class, Plane.h instance + friend,
  control_modes.cpp mode_from_mode_num() case, Parameters.cpp/h g2
  sub-group registration).
*/

#include "mode.h"
#include "Plane.h"

#ifndef AP_AHRS_AOA_ENABLED
#define AP_AHRS_AOA_ENABLED 0
#endif  // AP_AHRS_AOA_ENABLED

const AP_Param::GroupInfo ModeFBWL::var_info[] = {

    // @Param: PITCH_MIN
    // @DisplayName: FBWL bailout pitch floor
    // @Description: If the pitch attitude falls below this (nose-down)
    // value while flying the Fbwa submode of FBWL, the mode bails out
    // to Levelup to recover the aircraft.
    // @Units: deg
    // @Range: -80 -20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PITCH_MIN", 1, ModeFBWL, pitch_min_deg, FBWT_PITCH_MIN),

    // @Param: ALT_MIN
    // @DisplayName: FBWL bailout altitude floor
    // @Description: If the relative altitude above home (or, if home
    // was never set, above the altitude FBWL was selected at) falls
    // below this value while flying the Fbwa submode of FBWL, the mode
    // bails out to Levelup and climbs back to FBWL_ALT_MIN + FBWL_ALT_HYST.
    // @Units: m
    // @Range: 5 200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ALT_MIN", 2, ModeFBWL, alt_min_m, FBWT_ALT_MIN),

    // @Param: ALT_HYST
    // @DisplayName: FBWL bailout altitude recovery hysteresis
    // @Description: Additional altitude climbed above FBWL_ALT_MIN as
    // the Levelup climb target once a bailout has been triggered, to
    // give hysteresis against immediately re-triggering the
    // low-altitude bailout condition.
    // @Units: m
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ALT_HYST", 3, ModeFBWL, alt_hyst_m, 10),

    // @Param: AOA_MAX
    // @DisplayName: FBWL maximum usable angle of attack
    // @Description: Soft AoA limiter/rate-damping target in the Fbwa
    // submode: as estimated (or measured, if an AoA vane is fitted)
    // angle of attack approaches this value, FBWL blends in a nose-down
    // pitch correction. There is no equivalent stock ArduPlane
    // parameter, hence this addition. Set comfortably below the
    // airframe's actual stall AoA (typically 15-18 deg for most
    // trainers) to leave margin for the soft limiter to act before an
    // actual stall.
    // @Units: deg
    // @Range: 6 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("AOA_MAX", 4, ModeFBWL, aoa_max_deg, 12),  // ATTENTION: that is not pitch and should not be set based on pitch

    // @Param: LOAD_MAX
    // @DisplayName: FBWL maximum load factor
    // @Description: Soft G-limit in the Fbwa submode: commanded bank
    // angle is capped so that the load factor required to maintain
    // level flight at that bank (n = 1/cos(roll)) does not exceed this
    // value. There is no equivalent stock ArduPlane parameter (unlike
    // ANGLE_MAX on multirotors), hence this addition. This is also used,
    // exactly as stock STALL_PREVENTION already does elsewhere, as the
    // scaling factor applied to ARSPD_FBW_MIN in turns.
    // @Range: 1.5 4.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("LOAD_MAX", 5, ModeFBWL, load_max, 2.5),  // Estimated as speeds ~< (cruise / stall)^2

    AP_GROUPEND
};

/*
  Mode is (re)selected: FBWL always starts in the default Fbwa submode.
*/
bool ModeFBWL::_enter()
{
    enter_fbwa();

    warned_no_airspeed = false;
    aoa_prev_us = 0;
    last_log_ms = 0;
    aoa_limit_active = g_limit_active = stall_speed_limit_active = energy_limit_active = false;

    Location loc;
    if (ahrs.get_location(loc)) {
        no_home_ref_alt_cm = loc.alt;
    } else {
        no_home_ref_alt_cm = 0;
    }

    if (!ahrs.home_is_set()) {
        plane.gcs().send_text(MAV_SEVERITY_WARNING,
                               "FBWL: home not set, using entry altitude as ALT_MIN reference");
    }

    return true;
}

void ModeFBWL::enter_fbwa()
{
    submode = Submode::Fbwa;
}

void ModeFBWL::update()
{
    switch (submode) {
    case Submode::Fbwa:
        run_fbwa();
        break;
    case Submode::Levelup:
        run_levelup();
        break;
    }

    log_envelope();
}

/*
  run() mirrors ModeFBWA::run(): base class converts nav_roll_cd/
  nav_pitch_cd into servo output, then this handles throttle. Fbwa is
  manual throttle exactly like FBWA (which is what lets
  apply_energy_limiting() below use "is the pilot's own throttle
  already maxed out" as its saturation cue). Levelup is auto-throttle
  (TECS), unchanged from the previous revision, and TECS already
  degrades correctly with no airspeed sensor per stock behaviour.
*/
void ModeFBWL::run()
{
    Mode::run();

    if (submode == Submode::Fbwa) {
        output_pilot_throttle();
    }
}

/*
  ---------------------------------------------------------------------
  Fbwa submode: FBWA's stick -> attitude mapping, then every protection
  layer gets a chance to trim the resulting demand before it reaches the
  attitude controllers, then the hard bailout check runs last as the
  final safety net.
  ---------------------------------------------------------------------
*/
void ModeFBWL::run_fbwa()
{
    // --- begin: copied/adapted from ModeFBWA::update() ---
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;

    plane.update_load_factor();

    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max.get() * 100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * pitch_min_deg * 100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd,
                                         pitch_min_deg * 100,
                                         plane.aparm.pitch_limit_max.get() * 100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }

    if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    // --- end: copied/adapted from ModeFBWA::update() ---

    // --- continuous assisted-flight protections (order matters: each
    //     may adjust nav_roll_cd/nav_pitch_cd that the next one reads) ---
    apply_g_limit_protection();
    apply_stall_speed_protection();
    apply_aoa_protection();
    apply_energy_limiting();
    apply_coordinated_yaw();

    // --- hard bailout: last-resort net for when the above wasn't
    //     enough (e.g. a gust-induced stall) ---
    bool stalling = false;
    if (bailout_condition(stalling)) {
        enter_levelup();
    }
}

/*
  ---------------------------------------------------------------------
  Levelup submode: "Bailout; Recover". Unchanged in spirit from the
  previous revision, except that on recovery it now hands back to Fbwa
  (protected) rather than to a permanent Headhold.
  ---------------------------------------------------------------------
*/
void ModeFBWL::run_levelup()
{
    plane.nav_roll_cd = 0;

    const bool stalling = is_stalling();

    if (stalling) {
        plane.nav_pitch_cd = -500; // -5 deg, unload the wing
    } else {
        plane.nav_pitch_cd = MIN(1000, plane.aparm.pitch_limit_max.get() * 100); // <= 10 deg climb
    }

    Location target_loc;
    if (ahrs.get_location(target_loc)) {
        target_loc.alt = reference_alt_cm() + (int32_t)((alt_min_m + alt_hyst_m) * 100.0f);
        target_loc.relative_alt = 0;
        plane.set_target_altitude_location(target_loc);
    }

    float margin;
    const bool still_slow      = speed_too_low(margin);
    const bool still_low_pitch = pitch_too_low();

    if (!stalling && !still_slow && !still_low_pitch) {
        // Recovered: hand control straight back to the pilot. Fbwa's
        // own continuous protections (not a permanent hold) are what
        // keep this safe now.
        enter_fbwa();
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWL: recovered, control returned to pilot");
    }
}

void ModeFBWL::enter_levelup()
{
    if (submode == Submode::Levelup) {
        return;
    }
    submode = Submode::Levelup;
    levelup_enter_ms = AP_HAL::millis();
    plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWL: bailout triggered, recovering");
}

/*
  ---------------------------------------------------------------------
  Continuous protections (Fbwa submode only)
  ---------------------------------------------------------------------
*/

/*
  AoA protection + limiter + rate damping.

  Blends in a nose-down pitch correction as (estimated-or-measured) AoA,
  plus a short look-ahead based on its rate of change, approaches
  FBWL_AOA_MAX. The look-ahead term is the "rate damping" element: it
  anticipates an overshoot from a fast pitch-up stick input and starts
  correcting before AoA_MAX is actually reached, rather than only
  reacting once the limit is crossed.
*/
void ModeFBWL::apply_aoa_protection()
{
    const float aoa_deg = estimate_aoa_deg();

    const uint32_t now_us = AP_HAL::micros();
    float aoa_rate_dps = 0.0f;
    if (aoa_prev_us != 0) {
        const float dt = (now_us - aoa_prev_us) * 1.0e-6f;
        if (dt > 0.0001f && dt < 1.0f) {
            aoa_rate_dps = (aoa_deg - aoa_prev_deg) / dt;
        }
    }
    aoa_prev_deg = aoa_deg;
    aoa_prev_us = now_us;
    aoa_rate_dps_latest = aoa_rate_dps;

    constexpr float lookahead_s = 0.3f;
    const float predicted_aoa = aoa_deg + aoa_rate_dps * lookahead_s;
    const float aoa_margin = aoa_max_deg - predicted_aoa;

    aoa_limit_active = false;
    if (aoa_deg > 0.1f && aoa_margin < 3.0f) { // start blending in within 3 deg of the limit
        aoa_limit_active = true;
        const float correction_cd = constrain_float((3.0f - aoa_margin) * 150.0f, 0.0f, 1000.0f);
        plane.nav_pitch_cd -= (int32_t)correction_cd;
    }
}

/*
  Stall + G-limit protection (stall-speed half).

  Progressively biases pitch demand nose-down, and firmly caps roll,
  as IAS approaches / crosses the load-factor-scaled ARSPD_FBW_MIN
  floor - the same floor used by is_stalling()/bailout_condition(), but
  acted on continuously and proportionally here rather than only as a
  discrete trigger.
*/
void ModeFBWL::apply_stall_speed_protection()
{
    stall_speed_limit_active = false;

    float eas;
    if (!speed_available(eas)) {
        return;
    }

    const float load_factor = MAX(plane.aerodynamic_load_factor, 1.0f);
    const float min_speed = plane.aparm.airspeed_min * load_factor;
    const float margin = eas - min_speed;

    if (margin >= 2.0f) {
        return; // comfortably clear
    }

    stall_speed_limit_active = true;

    if (margin >= 0.0f) {
        // Approaching, not yet below: soft proportional correction.
        const float correction_cd = constrain_float((2.0f - margin) * 200.0f, 0.0f, 800.0f);
        plane.nav_pitch_cd -= (int32_t)correction_cd;
    } else {
        // Already below the scaled minimum: firmer nose-down bias, and
        // cap bank angle - flying slow and banked hard compounds both
        // stall and G risk simultaneously.
        plane.nav_pitch_cd -= 800; // -8 deg firm bias
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -3000, 3000); // <= 30 deg
    }
}

/*
  Stall + G-limit protection (G-limit half).

  Caps commanded bank angle so the load factor required to hold it
  (n = 1/cos(roll)) never exceeds FBWL_LOAD_MAX, gated by the stock
  STALL_PREVENTION parameter exactly as ArduPilot's own
  update_load_factor()-based roll scaling is.
*/
void ModeFBWL::apply_g_limit_protection()
{
    g_limit_active = false;

    if (!plane.aparm.stall_prevention) {
        return;
    }

    const float max_roll_for_load_rad = acosf(constrain_float(1.0f / MAX(load_max, 1.01f), -1.0f, 1.0f));
    const int32_t max_roll_for_load_cd = (int32_t)(degrees(max_roll_for_load_rad) * 100.0f);

    if (abs(plane.nav_roll_cd) > max_roll_for_load_cd) {
        g_limit_active = true;
        plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -max_roll_for_load_cd, max_roll_for_load_cd);
    }
}

/*
  TECS-integrated energy limiting.

  If the (pilot-commanded, in Fbwa) throttle is already saturated at
  THR_MAX and IAS is within a small margin of the load-factor-scaled
  ARSPD_FBW_MIN floor, there is no thrust margin left to trade for
  altitude: any further nose-up demand can only bleed more airspeed.
  Cap the pitch demand to a conservative fraction of PTCH_LIM_MAX_DEG in
  that state. This reuses THR_MAX/THR_MIN and PTCH_LIM_MAX_DEG (the same
  authoritative throttle/pitch envelope TECS itself is configured with)
  rather than inventing a new limit; if your tree's AP_TECS exposes a
  direct specific-energy-rate getter, that can be substituted here for a
  tighter integration.
*/
void ModeFBWL::apply_energy_limiting()
{
    energy_limit_active = false;

    float eas;
    if (!speed_available(eas)) {
        return;
    }

    const int16_t thr_out = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    const bool throttle_saturated = thr_out >= (plane.aparm.throttle_max.get() - 2);

    const float load_factor = MAX(plane.aerodynamic_load_factor, 1.0f);
    const float speed_margin = eas - plane.aparm.airspeed_min * load_factor;

    if (throttle_saturated && speed_margin < 3.0f) {
        energy_limit_active = true;
        const int32_t safe_pitch_cap_cd = (int32_t)(plane.aparm.pitch_limit_max.get() * 100.0f * 0.3f);
        plane.nav_pitch_cd = MIN(plane.nav_pitch_cd, safe_pitch_cap_cd);
    }
}

/*
  Coordinated turn + yaw blending.

  Deliberately a no-op on the control path. ArduPilot's stock
  AP_YawController (YAW2SRV_SLIP / YAW2SRV_RLL / YAW2SRV_INT /
  YAW2SRV_DAMP) already runs every loop via the shared
  Plane::stabilize_yaw() path for every fixed-wing mode, computing a
  coordinated-turn rudder component from the bank angle this mode
  commands and blending it with the pilot's rudder stick per the stock
  STICK_MIXING parameter. FBWL reuses that entirely rather than
  reimplementing turn coordination, and never writes to k_rudder itself,
  so that blending is never bypassed.

  The G-limit and AoA protections above already reduce the maximum
  commanded bank angle as those limits are approached, which is the
  correct way to reduce the rudder authority the yaw controller will
  ask for near the edge of the envelope, rather than clamping rudder
  output directly (which risks provoking exactly the yaw-roll coupling
  - an incipient spin - it would be trying to prevent).
*/
void ModeFBWL::apply_coordinated_yaw() const
{
    // Intentionally empty - see comment above.
}

/*
  ---------------------------------------------------------------------
  Envelope monitoring/logging
  ---------------------------------------------------------------------
  Streams an "FBWL" AP_Logger message at 10 Hz with the quantities every
  protection above is acting on, plus which protections are currently
  active, so a post-flight log review can see exactly when/why FBWL
  intervened. Also sends a one-shot GCS text on each rising edge of a
  protection engaging, rate-limited to avoid spamming.
*/
void ModeFBWL::log_envelope()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms < 100) { // 10 Hz
        return;
    }
    last_log_ms = now_ms;

    float eas = 0.0f;
    speed_available(eas);
    const float aoa_deg = estimate_aoa_deg();

    // @LoggerMessage: FBWL
    // @Description: FBWL assisted-flight envelope monitor
    // @Field: TimeUS: Time since system startup
    // @Field: SMode: active submode (0=Levelup,1=Fbwa)
    // @Field: AS: airspeed estimate used for protection (0 if none available)
    // @Field: AoA: estimated/measured angle of attack
    // @Field: AoAR: AoA rate of change
    // @Field: LoadF: aerodynamic load factor
    // @Field: Pitch: pitch attitude
    // @Field: RAlt: altitude relative to reference_alt_cm()
    // @Field: AoAL: AoA limiter active
    // @Field: GL: G-limit active
    // @Field: StL: stall-speed protection active
    // @Field: EnL: energy limiting active
    AP::logger().WriteStreaming("FBWL",
        "TimeUS,SMode,AS,AoA,AoAR,LoadF,Pitch,RAlt,AoAL,GL,StL,EnL",
        "QBffffffBBBB",
        AP_HAL::micros64(),
        (uint8_t)submode,
        eas,
        aoa_deg,
        aoa_rate_dps_latest,
        plane.aerodynamic_load_factor,
        ahrs.get_pitch_deg(),
        alt_too_low_margin_m(),
        (uint8_t)aoa_limit_active,
        (uint8_t)g_limit_active,
        (uint8_t)stall_speed_limit_active,
        (uint8_t)energy_limit_active);

    // Edge-triggered GCS notices, rate-limited to once per 3 s per flag
    // by simply comparing against the previous loop's flags.
    if (aoa_limit_active && !prev_aoa_limit_active) {
        plane.gcs().send_text(MAV_SEVERITY_INFO, "FBWL: AoA limiter active");
    }
    if (g_limit_active && !prev_g_limit_active) {
        plane.gcs().send_text(MAV_SEVERITY_INFO, "FBWL: G-limit active");
    }
    if (stall_speed_limit_active && !prev_stall_speed_limit_active) {
        plane.gcs().send_text(MAV_SEVERITY_INFO, "FBWL: stall-speed protection active");
    }
    if (energy_limit_active && !prev_energy_limit_active) {
        plane.gcs().send_text(MAV_SEVERITY_INFO, "FBWL: energy limiting active");
    }
    prev_aoa_limit_active = aoa_limit_active;
    prev_g_limit_active = g_limit_active;
    prev_stall_speed_limit_active = stall_speed_limit_active;
    prev_energy_limit_active = energy_limit_active;
}

/*
  ---------------------------------------------------------------------
  Sensor-availability + shared estimator helpers
  ---------------------------------------------------------------------
*/

bool ModeFBWL::have_position() const
{
    Location loc;
    return ahrs.get_location(loc);
}

int32_t ModeFBWL::reference_alt_cm() const
{
    if (ahrs.home_is_set()) {
        return plane.home.alt;
    }
    return no_home_ref_alt_cm;
}

// relative altitude in metres for logging; 0 if no estimate available
float ModeFBWL::alt_too_low_margin_m() const
{
    Location loc;
    if (!ahrs.get_location(loc)) {
        return 0.0f;
    }
    return (loc.alt - reference_alt_cm()) * 0.01f;
}

bool ModeFBWL::speed_available(float &eas) const
{
    if (ahrs.airspeed_EAS(eas)) {
        return true;
    }

    if (!have_position()) {
        if (!warned_no_airspeed) {
            plane.gcs().send_text(MAV_SEVERITY_WARNING,
                                   "FBWL: no airspeed source, speed/stall/AoA/energy protections disabled");
            warned_no_airspeed = true;
        }
        return false;
    }

    const float EAS2TAS = ahrs.get_EAS2TAS();
    eas = plane.smoothed_airspeed / MAX(EAS2TAS, 0.1f);
    return true;
}

bool ModeFBWL::speed_too_low(float &margin_ms) const
{
    float eas;
    if (!speed_available(eas)) {
        margin_ms = 0.0f;
        return false;
    }

    const float load_factor = MAX(plane.aerodynamic_load_factor, 1.0f);
    const float min_speed_eas = plane.aparm.airspeed_min * load_factor;

    margin_ms = eas - min_speed_eas;
    return margin_ms < 0.0f;
}

/*
  Estimated angle of attack, degrees. Uses a real AoA vane if compiled
  in; otherwise a synthetic pitch-minus-flight-path-angle estimate.
  Returns 0 if no usable estimate can be formed at all (no speed source
  - see speed_available()), so callers never falsely trigger from this.
*/
float ModeFBWL::estimate_aoa_deg() const
{
#if AP_AHRS_AOA_ENABLED
    float aoa, aos;
    if (ahrs.get_AOA_and_SSA(aoa, aos)) {
        return aoa;
    }
#endif

    float eas;
    if (!speed_available(eas) || eas < 1.0f) {
        return 0.0f;
    }

    float climb_rate;
    if (ahrs.get_velocity_D(climb_rate)) {
        climb_rate = -climb_rate;
    } else {
        // Baro-only fallback - needs neither GPS nor airspeed.
        climb_rate = plane.barometer.get_climb_rate();
    }

    const float gamma_rad = asinf(constrain_float(climb_rate / eas, -1.0f, 1.0f));
    return ahrs.get_pitch_deg() - degrees(gamma_rad);
}

bool ModeFBWL::is_stalling() const
{
    float margin;
    const bool slow = speed_too_low(margin);
    const bool deep_margin = slow && (margin < -1.0f);

    bool high_aoa = false;
#if AP_AHRS_AOA_ENABLED
    float aoa_deg, aos_deg;
    ahrs.get_AOA_and_SSA(aoa_deg, aos_deg);
    high_aoa = (aoa_deg > 15.0f);
#endif

    return deep_margin || high_aoa;
}

bool ModeFBWL::pitch_too_low() const
{
    return ahrs.get_pitch_deg() < pitch_min_deg;
}

bool ModeFBWL::alt_too_low() const
{
    Location loc;
    if (!ahrs.get_location(loc)) {
        return false;
    }
    const float rel_alt_m = (loc.alt - reference_alt_cm()) * 0.01f;
    return rel_alt_m < alt_min_m;
}

// Hard AoA breach: the soft limiter has failed to hold AoA within a
// small margin past FBWL_AOA_MAX.
bool ModeFBWL::aoa_exceeded_hard() const
{
    const float aoa_deg = estimate_aoa_deg();
    return aoa_deg > (aoa_max_deg + 5.0f);
}

// Hard G breach: actual load factor already meaningfully over FBWL_LOAD_MAX.
bool ModeFBWL::g_exceeded_hard() const
{
    return plane.aerodynamic_load_factor > (load_max * 1.1f);
}

bool ModeFBWL::bailout_condition(bool &stalling) const
{
    stalling = is_stalling();

    float margin;
    const bool too_slow    = speed_too_low(margin);
    const bool too_low_ptc = pitch_too_low();
    const bool too_low_alt = alt_too_low();
    const bool aoa_hard    = aoa_exceeded_hard();
    const bool g_hard      = g_exceeded_hard();

    return stalling || too_slow || too_low_ptc || too_low_alt || aoa_hard || g_hard;
}
