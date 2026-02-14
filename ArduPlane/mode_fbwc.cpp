#include "mode.h"
#include "Plane.h"

bool ModeFBWC::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    if (!AP::ahrs().healthy()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX: AHRS is not healthy, FBWC is unstable");
        // Note: It makes sense to allow FBWC even for unhealthy AHRS, moving the responsibility to pilot
        // return false;
    }
    plane.set_target_altitude_current();
    target_yaw = AP::ahrs().get_yaw_rad();
    
    return true;
}

void ModeFBWC::update()
{
    plane.nav_roll_cd = ((target_yaw - AP::ahrs().get_yaw_rad()) / M_PI) * plane.roll_limit_cd;
    plane.update_load_factor();
    // Handle speed and height control in FBWC, similar to FBWB, however altitude and direction are not taken from user input being fixed on level flight at current heading
    plane.update_fbwb_speed_height(true);
}
