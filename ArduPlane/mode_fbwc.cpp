#include "mode.h"
#include "Plane.h"

bool ModeFBWC::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();
    target_yaw = plane.ahrs.get_yaw();
    
    return true;
}

void ModeFBWC::update()
{
    plane.nav_roll_cd = ((target_yaw - plane.ahrs.get_yaw()) / M_PI) * plane.roll_limit_cd;
    plane.update_load_factor();
    // Handle speed and height control in FBWC, similar to FBWB, however altitude and direction are not taken from user input being fixed on level flight at current heading
    plane.update_fbwb_speed_height(true);
}
