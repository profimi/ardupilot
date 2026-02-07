/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_VideoTX.h"

#if AP_VIDEOTX_ENABLED

#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/AP_HAL.h>

#include <algorithm>

extern const AP_HAL::HAL& hal;

AP_VideoTX *AP_VideoTX::singleton;

const AP_Param::GroupInfo AP_VideoTX::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Is the Video Transmitter enabled or not
    // @Description: Toggles the Video Transmitter on and off
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_VideoTX, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: POWER
    // @DisplayName: Video Transmitter Power Level
    // @Description: Video Transmitter Power Level. Different VTXs support different power levels, the power level chosen will be rounded down to the nearest supported power level
    // @Range: 1 1000
    AP_GROUPINFO("POWER",    2, AP_VideoTX, _power_mw, 0),

    // @Param: CHANNEL
    // @DisplayName: Video Transmitter Channel
    // @Description: Video Transmitter Channel
    // @User: Standard
    // @Range: 0 7
    AP_GROUPINFO("CHANNEL",  3, AP_VideoTX, _channel, 0),

    // @Param: BAND
    // @DisplayName: Video Transmitter Band
    // @Description: Video Transmitter Band
    // @User: Standard
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand,6:1G3 Band A,7:1G3 Band B,8:Band X,9:3G3 Band A,10:3G3 Band B
    AP_GROUPINFO("BAND",  4, AP_VideoTX, _band, 0),

    // @Param: FREQ
    // @DisplayName: Video Transmitter Frequency
    // @Description: Video Transmitter Frequency. The frequency is derived from the setting of BAND and CHANNEL
    // @User: Standard
    // @ReadOnly: True
    // @Range: 1000 6100
    AP_GROUPINFO("FREQ",  5, AP_VideoTX, _frequency_mhz, 0),

    // @Param: OPTIONS
    // @DisplayName: Video Transmitter Options
    // @Description: Video Transmitter Options. Pitmode puts the VTX in a low power state. Unlocked enables certain restricted frequencies and power levels. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels. One stop-bit may be required for VTXs that erroneously mimic iNav behaviour.
    // @User: Advanced
    // @Bitmask: 0:Pitmode,1:Pitmode until armed,2:Pitmode when disarmed,3:Unlocked,4:Add leading zero byte to requests,5:Use 1 stop-bit in SmartAudio,6:Ignore CRC in SmartAudio,7:Ignore status updates in CRSF and blindly set VTX options
    AP_GROUPINFO("OPTIONS",  6, AP_VideoTX, _options, 0),

    // @Param: MAX_POWER
    // @DisplayName: Video Transmitter Max Power Level
    // @Description: Video Transmitter Maximum Power Level. Different VTXs support different power levels, this prevents the power aux switch from requesting too high a power level. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting.
    // @Range: 25 10000
    AP_GROUPINFO("MAX_POWER", 7, AP_VideoTX, _max_power_mw, 2500),

    // Presets //////////////////////////////////////////////////

    // @Param: PRESET1
    // @DisplayName: Preset #1
    // @Description: VTX preset, in form XY where X is band and Y is channel. E.g. 02 means A-band, 3-d channel
    // Range: (MAX_BANDS - 1)*10 + (BAND_CHANNELS_NUM - 1) = 167 < 317 ((2^5-1)*10 + 2^3-1)
    // @Range: 0 317
    AP_GROUPINFO("PRESET1", 8, AP_VideoTX, _preset[0], 00),

    // @Param: PRESET2
    // @DisplayName: Preset #2
    // @Description: VTX preset, in form XY where X is band and Y is channel. E.g. 02 means A-band, 3-d channel
    // @Range: 0 317
    AP_GROUPINFO("PRESET2", 9, AP_VideoTX, _preset[1], 01),

    // @Param: PRESET3
    // @DisplayName: Preset #3
    // @Description: VTX preset, in form XY where X is band and Y is channel. E.g. 02 means A-band, 3-d channel
    // @Range: 0 317
    AP_GROUPINFO("PRESET3", 10, AP_VideoTX, _preset[2], 02),

    // @Param: PRESET4
    // @DisplayName: Preset #4
    // @Description: VTX preset, in form XY where X is band and Y is channel. E.g. 02 means A-band, 3-d channel
    // @Range: 0 317
    AP_GROUPINFO("PRESET4", 11, AP_VideoTX, _preset[3], 03),

    // @Param: PRESET5
    // @DisplayName: Preset #5
    // @Description: VTX preset, in form XY where X is band and Y is channel. E.g. 02 means A-band, 3-d channel
    // @Range: 0 317
    AP_GROUPINFO("PRESET5", 12, AP_VideoTX, _preset[4], 04),

    // @Param: PRESET6
    // @DisplayName: Preset #6
    // @Description: VTX preset, in form XY where X is band and Y is channel. E.g. 02 means A-band, 3-d channel
    // @Range: 0 317
    AP_GROUPINFO("PRESET6", 13, AP_VideoTX, _preset[5], 05),

    // @Param: MODEL
    // @DisplayName: VTX Model
    // @Description: VTX Model: 0 generic,  D1, ...
    // @Range: 0 9
    AP_GROUPINFO("MODEL", 14, AP_VideoTX, _model, 1),

    // @Param: POW_LEVELS
    // @DisplayName: Power level count
    // @Description: How many proper power levels has been configured, <= VTX_MAX_ADJUSTABLE_POWER_LEVELS <= 8 (max npos switch values)
    // @Range: 0 VTX_MAX_ADJUSTABLE_POWER_LEVELS
    AP_GROUPINFO("POW_LEVELS", 15, AP_VideoTX, _num_active_levels, 6),  // 6 to use 6pos or rotational switch

    // @Param: POW_CVAL1
    // @DisplayName: VTX custom power value
    // @Description: VTX custom power values specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CVAL1", 16, AP_VideoTX, _cvals[0], 0),

    // @Param: POW_CVAL2
    // @DisplayName: VTX custom power value
    // @Description: VTX custom power values specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CVAL2", 17, AP_VideoTX, _cvals[1], 1),

    // @Param: POW_CVAL3
    // @DisplayName: VTX custom power value
    // @Description: VTX custom power values specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CVAL3", 18, AP_VideoTX, _cvals[2], 2),

    // @Param: POW_CVAL4
    // @DisplayName: VTX custom power value
    // @Description: VTX custom power values specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CVAL4", 19, AP_VideoTX, _cvals[3], 3),

    // @Param: POW_CVAL5
    // @DisplayName: VTX custom power value
    // @Description: VTX custom power values specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CVAL5", 20, AP_VideoTX, _cvals[4], 4),

    // @Param: POW_CVAL6
    // @DisplayName: VTX custom power value
    // @Description: VTX custom power values specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CVAL6", 21, AP_VideoTX, _cvals[5], 5),

    // @Param: POW_CMW1
    // @DisplayName: VTX custom power in mW
    // @Description: VTX custom power in mW specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CMW1", 22, AP_VideoTX, _cmws[0], 0),

    // @Param: POW_CMW2
    // @DisplayName: VTX custom power in mW
    // @Description: VTX custom power in mW specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CMW2", 23, AP_VideoTX, _cmws[1], 0),

    // @Param: POW_CMW3
    // @DisplayName: VTX custom power in mW
    // @Description: VTX custom power in mW specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CMW3", 24, AP_VideoTX, _cmws[2], 0),

    // @Param: POW_CMW4
    // @DisplayName: VTX custom power in mW
    // @Description: VTX custom power in mW specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CMW4", 25, AP_VideoTX, _cmws[3], 0),

    // @Param: POW_CMW5
    // @DisplayName: VTX custom power in mW
    // @Description: VTX custom power in mW specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CMW5", 26, AP_VideoTX, _cmws[4], 0),

    // @Param: POW_CMW6
    // @DisplayName: VTX custom power in mW
    // @Description: VTX custom power in mW specified by the hardware producer
    // @Range: 0 32767
    AP_GROUPINFO("POW_CMW6", 27, AP_VideoTX, _cmws[5], 0),

    // @Param: USER_FREQ
    // @DisplayName: User-specified frequency (Ardupilot's VTX table) prioritization over the VTX internal one
    // @Description: Whether to prioritize the user-specified frequency over the one corresponding to the Band/Channel in the internal VTX table
    // @Values: 0:Use internal VTX band/channel mapping table, 1: Enforce user the specified frequency over the band/channel VTX-internal mapping
    AP_GROUPINFO("USER_FREQ", 28, AP_VideoTX, _user_freq, 1),

    AP_GROUPEND
};

// #define VTX_DEBUG
#ifdef VTX_DEBUG
# define debug(fmt, args...)	hal.console->printf("VTX: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL& hal;

// Mapping of power level to milli watt to dbm
// valid power levels from SmartAudio spec, the adjacent levels might be the actual values
// so these are marked as level + 0x10 and will be switched if a dbm message proves it
// Ascedenting ordering of this table by the power in mw is essential
// D1 Note: power switching works for SamertAudio and fails for the original IRC Tramp that uses power_mw value,
// where D1 requires power_dbm value
// AP_VideoTX::PowerLevel AP_VideoTX::_power_levels[VTX_MAX_POWER_LEVELS] = {
//
// NOTE: level  - active power levels enumeration starting from 0, other entries are marked as inactive.
// 0x1N active power levels are automatically reassigend as 0x0N
// ATTENTION: this table should have ascendant ordering by mW and dBm;
// level is actual only for the SmartAudio v1.0/2.0, dac only for the SmartAudio v1.0
AP_VideoTX::PowerLevel AP_VideoTX::_power_levels[] = {
    // level, mw, dbm, dac
    { 0x10, 0,    0, 0   }, // only in SA 2.1
    { 0x20, 10,   10, 5  }, // TBS_UPD
    { 0,    25,   14, 7  }, // D1; TBS_UPD
    { 0x11, 100,  20, 10 }, // only in SA 2.1; TBS_UPD
    { 1,    200,  23, 16 }, // TBS_UPD
    { 0x12, 400,  26, 20 }, // only in SA 2.1; TBS_UPD
    { 2,    500,  27, 25 }, // D1; TBS_UPD
    { 0x22, 600,  28, 30 },
    { 3,    800,  29, 40 },
    { 0x13, 1000, 30, 50 }, // only in SA 2.1; D1; TBS_UPD
    { 4,    1200, 31, 52 },
    { 0x14, 1600, 32, 56 },
    { 5,    2000, 33, 60 },
    { 0x15, 2500, 34, 65 }, // D1
    { 0x16, 3000, 35, 70 }, // AKK8/5/3; TBS 3W
    { 0xFF, 0,    0,  0XFF, PowerActive::Inactive }  // slot reserved for a custom power level
};

const uint8_t VTX_MAX_POWER_LEVELS = sizeof(AP_VideoTX::_power_levels) / sizeof(AP_VideoTX::_power_levels[0]);
static_assert(VTX_MAX_POWER_LEVELS >= VTX_MAX_ADJUSTABLE_POWER_LEVELS, "VTX_MAX_ADJUSTABLE_POWER_LEVELS is out of range");

// D1 => _num_active_levels = 4:  25, 500, 1000, 2500; _max_power_mw = 2500
// FXR10 => _num_active_levels = 5 (6):  500, 2500, 5000, 7500, 10000; _max_power_mw = 10000
// AKK5 => _num_active_levels = 6:  25, 200, 500, 1000, 3000, 5000; _max_power_mw = 5000

// dBm by mW: \(P_{dBm}=10\log _{10}(\frac{P}{1mW})\)

// AKK power levels
// 25/250/500/1000/2000/3000mW
// 200 400 800 1600
// 25 200 600 1200

// // Original VTX values from Ardupilot master
// AP_VideoTX::PowerLevel AP_VideoTX::_power_levels[VTX_MAX_POWER_LEVELS] = {
//     // level, mw, dbm, dac
//     { 0xFF,  0,    0, 0    }, // only in SA 2.1
//     { 0,    25,   14, 7    },
//     { 0x11, 100,  20, 0xFF }, // only in SA 2.1
//     { 1,    200,  23, 16   },
//     { 0x12, 400,  26, 0xFF }, // only in SA 2.1
//     { 2,    500,  27, 25   },
//     { 0x12, 600,  28, 0xFF }, // Tramp lies above power levels and always returns 25/100/200/400/600
//     { 3,    800,  29, 40   },
//     { 0x13, 1000, 30, 0xFF }, // only in SA 2.1
//     { 0xFF, 0,    0,  0XFF, PowerActive::Inactive }  // slot reserved for a custom power level
// };

AP_VideoTX::AP_VideoTX()
{
    if (singleton) {
        AP_HAL::panic("Too many VTXs");
        return;
    }
    singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

AP_VideoTX::~AP_VideoTX(void)
{
    singleton = nullptr;
}

bool AP_VideoTX::init(void)
{
    if (_initialized)
        return false;

    // PARAMETER_CONVERSION - Added: Sept-2022
    _options.convert_parameter_width(AP_PARAM_INT16);

    // Correct static tables to match object parameters
    // And sync RC_Channel::read_npos_switch levels with _num_active_levels
    syncActiveLevs(_num_active_levels > VTX_MAX_ADJUSTABLE_POWER_LEVELS
        ? VTX_MAX_ADJUSTABLE_POWER_LEVELS : _num_active_levels);

    /*! @brief Power levels initialization
    * @param[in] pwrMax  - the number of power levels
    * @param[in] mws  - milli Watt values
    * @param[in] doEnum  - whether to reset and enumerate the level values corresponding to those power values,
    *   which is essential for SmartAudio 2.0
    */
    auto initPowerLevels = [this](uint16_t pwrMax, std::initializer_list<uint16_t>&& mws, bool doEnum=true)
    {
        _max_power_mw.set_and_save(pwrMax);
        syncActiveLevs(mws.size());
        uint8_t i = 0, n = 0;
        for(auto mw: mws) {
            for(; i < VTX_MAX_POWER_LEVELS; ++i) {
                if(_power_levels[i].mw < mw) {
                    _power_levels[i].active = PowerActive::Inactive;
                    _power_levels[i].level = 0xFF;  // Invalidate the power level
                } else {
                    if(_power_levels[i].mw == mw) {
                        if(doEnum)
                            _power_levels[i].level = n++;
                        ++i;
                    } else GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VTX power list lacks predefined level: %u mW", mw);
                    break;
                }
            }
        }
        // Invalidate the remained levels
        for(; i < VTX_MAX_POWER_LEVELS; ++i) {
            _power_levels[i].active = PowerActive::Inactive;
            _power_levels[i].level = 0xFF;  // Invalidate the power level
        }
    };

    // Init freqMap with the actual parameters
    _freqMap.init(model(), is_user_freq());

    // Make inactive power levels exceeding the power capacity of the target VTX
    switch (model()) {
    case Model::D1:
        initPowerLevels(2500, {25, 500, 1000, 2500});
        break;
    case Model::TBS_UPD:
        initPowerLevels(1000, {10, 25, 100, 200, 500, 1000});  // 400
        break;
    case Model::D1_P3:
        initPowerLevels(2500, {25, 1000, 2500});
        break;
    case Model::TBS_UPD_P3:
        initPowerLevels(3000, {25, 1000, 3000});
        break;
    case Model::TBS_UPD3:
        initPowerLevels(3000, {25, 100, 200, 500, 1000, 3000});  // 25, 100, 200, 400, 1000, 3000
        break;
    case Model::CUSTOM:
        for(uint8_t i = 0; i < _num_active_levels; ++i) {
            _power_vals[i].val = _cvals[i];
            _power_vals[i].mw = _cmws[i];
        }
        validate_cpowlevs();
        break;
    default:
        // Consider _max_power_mw
        for(uint8_t i = VTX_MAX_POWER_LEVELS - 1; i > 0; --i) {
            if(_power_levels[i].active != PowerActive::Inactive) {
                if(_power_levels[i].mw > _max_power_mw) {
                    _power_levels[i].active = PowerActive::Inactive;
                    _power_levels[i].level = 0xFF;  // Invalidate the power level
                }
                else break;
            }
        }
    }

    // Find the index into the power table
    _current_power = 0;
    while(_current_power < VTX_MAX_POWER_LEVELS && _power_levels[_current_power].mw < _power_mw)
        ++_current_power;
    if(_current_power && _power_levels[_current_power].mw > _power_mw)
        --_current_power;
    _power_mw.set_and_save(get_power_mw());

    _current_frequency = _frequency_mhz;
    _current_band = _band;
    _current_channel = _channel;
    _current_options = _options;
    _current_enabled = _enabled;
    _initialized = true;

    return true;
}

bool AP_VideoTX::syncActiveLevs(uint8_t num)
{
    // Set respective npos switch to this number of positions
    // Find_channel_for_option() returns a pointer to the assigned RC_Channel
    bool res = false;
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::VTX_POWER);
    if (chan != nullptr) {
        // // Use chan->ch() to get the 1-indexed channel number (e.g., 9 for RC9)
        // uint8_t channel_num = chan->ch();
        res = chan->set_npos_switch_levels(num);
    } else GCS_SEND_TEXT(MAV_SEVERITY_INFO, "No RC channel assigned to VTX Power");
    if (res)
        _num_active_levels.set_and_save(num);
    else GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "VTX power levels count != npos switch levels");
    return res;
}

bool AP_VideoTX::get_band_and_channel(uint16_t freq, VideoBand& band, uint8_t& channel) const
{
    const bool approx = AP::vtx()._user_freq;
    const uint8_t bands = bands_num();
    uint16_t df = -1;  // Error in frequency band/channel identification, -1 is the max value

    for (uint8_t i = 0; i < bands; ++i) {
        for (uint8_t j = 0; j < BAND_CHANNELS_NUM; ++j) {
            if (get_frequency_mhz(i, j) == freq || (approx && df > abs(get_frequency_mhz(i, j) - freq))) {
                band = VideoBand(i);
                channel = j;
                if(approx && get_frequency_mhz(i, j) != freq)
                    df = abs(get_frequency_mhz(i, j) - freq);
                else return true;
            }
        }
    }
    return approx;
}

// set the current power
void AP_VideoTX::set_configured_power_mw(uint16_t power)
{
    _power_mw.set_and_save_ifchanged(power);
}

uint8_t AP_VideoTX::find_current_power() const
{
    if(_current_power < VTX_MAX_POWER_LEVELS && _power_mw == _power_levels[_current_power].mw)
        return _current_power;

    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; ++i) {
        if (_power_mw == _power_levels[i].mw)
            return i;
    }
    return 0;
}

uint16_t AP_VideoTX::power_at_lev(uint8_t lev, uint8_t beg) const
{
    for (uint8_t i = beg; i < VTX_MAX_POWER_LEVELS; ++i)
        // Note: there might be several levels with the same value, but only the first active one is actual
        if(lev == (_power_levels[i].level & 0xF) && _power_levels[_current_power].active == PowerActive::Active)
            return _power_levels[i].mw;
    return 0;
}

// set the power in dbm, rounding appropriately
void AP_VideoTX::set_power_dbm(uint8_t power, PowerActive active)
{
    if (power == _power_levels[_current_power].dbm
        && _power_levels[_current_power].active == active) {
        return;
    }

    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (power == _power_levels[i].dbm) {
            _current_power = i;
            _power_levels[i].active = active;
            debug("learned power %ddbm", power);
            return;
        }
    }
    // learn the non-standard power
    _current_power = update_power_dbm(power, active);
}

// add an active power setting in dbm starting the search from the index i
uint8_t AP_VideoTX::update_power_dbm(uint8_t power, PowerActive active, uint8_t i)
{
    if(i >= VTX_MAX_POWER_LEVELS)
        i = 0;
    for (; i < VTX_MAX_POWER_LEVELS && _power_levels[i].dbm <= power; ++i) {
        if (power == _power_levels[i].dbm) {
            if (_power_levels[i].active != active) {
                _power_levels[i].active = active;
                debug("%s power %ddbm", active == PowerActive::Active ? "learned" : "invalidated", power);
            }
            return i;
        }
    }
    // Insert new power levels if necessary
    if(i < VTX_MAX_POWER_LEVELS) {
        // Move the previous value to the reserved custom slot; However that invalidates the ordering
        // if(i < VTX_MAX_POWER_LEVELS-1)
        //     _power_levels[VTX_MAX_POWER_LEVELS-1] = _power_levels[i];
        _power_levels[i].dbm = power;
        // _power_levels[i].level = 0xFF;  // Retain the level number
        _power_levels[i].dac = 0xFF;
        _power_levels[i].mw = uint16_t(roundf(powf(10, power * 0.1f)));
        _power_levels[i].active = active;
        debug("non-standard power %ddbm -> %dmw", power, _power_levels[i].mw);
    }
    return i;
}

// add all active power setting in dbm
void AP_VideoTX::update_all_power_dbm(uint8_t nlevels, const uint8_t power[])
{
    if (nlevels > VTX_MAX_POWER_LEVELS)
        nlevels = VTX_MAX_POWER_LEVELS;
    for (uint8_t i = 0, j = i; i < nlevels && j < VTX_MAX_POWER_LEVELS; ++i, ++j) {
        j = update_power_dbm(power[i], PowerActive::Active, j);
        _power_levels[j].level = i;
    }
    // invalidate the remaining ones
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; ++i)
        if (_power_levels[i].active == PowerActive::Unknown) {
            _power_levels[i].active = PowerActive::Inactive;
            _power_levels[i].level = 0xFF;
        }
}

// set the power in mw
void AP_VideoTX::set_power_mw(uint16_t power)
{
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS && power >= _power_levels[i].mw; ++i) {
        if (power == _power_levels[i].mw) {
            _current_power = i;
            break;
        }
    }
}

// set the power "level"
void AP_VideoTX::set_power_level(uint8_t level, PowerActive active)
{
    if (level == _power_levels[_current_power].level
        && _power_levels[_current_power].active == active) {
        return;
    }

    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (level == _power_levels[i].level) {
            _current_power = i;
            _power_levels[i].active = active;
            debug("learned power level %d: %dmw", level, get_power_mw());
            break;
        }
    }
}

// set the power dac
void AP_VideoTX::set_power_dac(uint16_t power, PowerActive active)
{
    if (power == _power_levels[_current_power].dac
        && _power_levels[_current_power].active == active) {
        return;
    }

    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (power == _power_levels[i].dac) {
            _current_power = i;
            _power_levels[i].active = active;
            debug("learned power %dmw", get_power_mw());
        }
    }
}

// Validate custom power levels by deactivating non-specified once
void AP_VideoTX::validate_cpowlevs(bool doEnum)
{
    uint8_t  j = 0;
    for(uint8_t  i = 0; i < VTX_MAX_POWER_LEVELS; ++i) {
        if(j >= _num_active_levels || _power_levels[i].mw < _power_vals[j].mw) {
            _power_levels[i].active = PowerActive::Inactive;
            _power_levels[i].level = 0xFF;  // Invalidate the power level
        } else {
            if(_power_vals[j].mw == _power_levels[i].mw && doEnum)
                _power_levels[i].level = j;
            ++j;
        }
    }
}

// Set power value (custom or predefined)
void AP_VideoTX::set_power_val(uint16_t power, PowerActive active)
{
    // Get custom mW by the value, use approximate value if the exact one has not been found
    auto cmw = [this](uint16_t val) {
        uint8_t i = 0;
        for (; i < _num_active_levels && _power_vals[i].val <= val; ++i)
            if (val == _power_vals[i].val)
                return _power_vals[i].mw;
        if (i >= _num_active_levels || (i > 0 && _power_vals[i].mw - val > val - _power_vals[i-1].mw))
            --i;
        return _power_vals[i].mw;
    };

    if (cmw(power) == _power_levels[_current_power].mw
    && _power_levels[_current_power].active == active)
        return;

    for (uint8_t i = 0, j = 0; i < VTX_MAX_POWER_LEVELS && j < _num_active_levels;) {
        if (_power_levels[i].mw >= _power_vals[j].mw) {
            if (power == _power_vals[j].val) {
                _current_power = i;
                _power_levels[i].active = active;
                debug("learned power %dmw", get_power_mw());
                break;
            } else ++j;
        } else ++i;
    }
}

uint16_t AP_VideoTX::get_configured_power_val() const
{
    // Note: _num_active_levels <= VTX_MAX_ADJUSTABLE_POWER_LEVELS
    for(uint8_t i = 0; i < _num_active_levels && _power_vals[i].mw <= _power_mw; ++i)
        if(_power_vals[i].mw == _power_mw)
            return _power_vals[i].val;
    return 0;
}

// set the current channel
void AP_VideoTX::set_enabled(bool enabled)
{
    _current_enabled = enabled;
    if (!_enabled.configured()) {
        _enabled.set_and_save(enabled);
    }
}

void AP_VideoTX::set_power_is_current()
{
    set_power_dbm(get_configured_power_dbm());
}

void AP_VideoTX::set_freq_is_current()
{
    _current_frequency = _frequency_mhz;
    _current_band = _band;
    _current_channel = _channel;
}

// periodic update
void AP_VideoTX::update(void)
{
    if (!_enabled) {
        return;
    }

    // manipulate pitmode if pitmode-on-disarm or power-on-arm is set
    if (has_option(VideoOptions::VTX_PITMODE_ON_DISARM) || has_option(VideoOptions::VTX_PITMODE_UNTIL_ARM)) {
        if (hal.util->get_soft_armed() && has_option(VideoOptions::VTX_PITMODE)) {
            _options.set(_options & ~uint8_t(VideoOptions::VTX_PITMODE));
        } else if (!hal.util->get_soft_armed() && !has_option(VideoOptions::VTX_PITMODE)
            && has_option(VideoOptions::VTX_PITMODE_ON_DISARM)) {
            _options.set(_options | uint8_t(VideoOptions::VTX_PITMODE));
        }
    }
    // check that the requested power is actually allowed
    // reset if not
    if (_power_mw != get_power_mw()) {
        if (_power_levels[find_current_power()].active == PowerActive::Inactive) {
            // reset to something we know works
            debug("power reset to %dmw from %dmw", get_power_mw(), _power_mw.get());
            _power_mw.set_and_save(get_power_mw());
        }
    }
}

bool AP_VideoTX::update_options() const
{
    if (!_defaults_set)
        return false;
    // check pitmode
    if ((_options & uint8_t(VideoOptions::VTX_PITMODE))
    != (_current_options & uint8_t(VideoOptions::VTX_PITMODE)))
        return true;

#if HAL_CRSF_TELEM_ENABLED
    // using CRSF so unlock is not an option
    if (AP::crsf_telem() != nullptr)
        return false;
#endif
    // check unlock only
    if ((_options & uint8_t(VideoOptions::VTX_UNLOCKED)) != 0
    && (_current_options & uint8_t(VideoOptions::VTX_UNLOCKED)) == 0)
        return true;

    // ignore everything else
    return false;
}

void AP_VideoTX::set_preset(uint8_t preset_no)
{
    // assert(preset_no < sizeof _preset && "preset_no is out of range");
    if(preset_no >= sizeof _preset) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Out of range, omitting preset_no: %u (>= %u)", preset_no, sizeof _preset);
        return;
    }
    // Note: heximal instead of the decimal digit system is used to cover up to 16 bands
    set_band(_preset[preset_no] / 10);
    set_channel(_preset[preset_no] % 10);
}

bool AP_VideoTX::update_power() const {
    if (!_defaults_set || _power_mw == get_power_mw() || get_pitmode())
        return false;
    // check that the requested power is actually allowed
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS && _power_mw >= _power_levels[i].mw; i++) {
        if (_power_mw == _power_levels[i].mw
        && _power_levels[i].active != PowerActive::Inactive)
            return true;
    }
    // asked for something unsupported - only SA2.1 allows this and will have already provided a list
    return false;
}

bool AP_VideoTX::have_params_changed() const
{
    return _enabled
        && (update_power()
        || update_band()
        || update_channel()
        || update_frequency()
        || update_options());
}

// update the configured frequency to match the channel and band
void AP_VideoTX::update_configured_frequency()
{
    _frequency_mhz.set_and_save(get_frequency_mhz(_band, _channel));
}

// update the configured channel and band to match the frequency
// ATTENTION: updates _frequency_mhz by the current band and channel if it cannot be used to define the them
void AP_VideoTX::update_configured_channel_and_band()
{
    VideoBand band;
    uint8_t channel;
    if (get_band_and_channel(_frequency_mhz, band, channel)) {
        _band.set_and_save(band);
        _channel.set_and_save(channel);
    } else update_configured_frequency();  // sets _frequency_mhz
}

// set the current configured values if not currently set in storage
// this is necessary so that the current settings can be seen
bool AP_VideoTX::set_defaults()
{
    if (_defaults_set)
        return false;

    // check that our current view of frequency matches band/channel of the HW VTX mapping
    // if not then force one to be correct and correspond to our mapping
    uint16_t calced_freq = get_frequency_mhz(_current_band, _current_channel);
    if (_current_frequency != calced_freq) {
        VideoBand band;
        uint8_t channel;
        if (_current_frequency > 0 && get_band_and_channel(_current_frequency, band, channel)) {
            _current_band = band;
            _current_channel = channel;
        } else _current_frequency = calced_freq;
    }

    if (!_options.configured())
        _options.set_and_save(_current_options);
    if (!_channel.configured())
        _channel.set_and_save(_current_channel);
    if (!_band.configured())
        _band.set_and_save(_current_band);
    if (!_power_mw.configured())
        _power_mw.set_and_save(get_power_mw());
    if (!_user_freq && !_frequency_mhz.configured())
        _frequency_mhz.set_and_save(_current_frequency);

    // Now check that the user didn't screw up by selecting incompatible options
    if (_frequency_mhz != get_frequency_mhz(_band, _channel)) {
        if (_frequency_mhz > 0)
            update_configured_channel_and_band();  // Note: might update _frequency_mhz if it is not preset in the mapping table 
        else update_configured_frequency();  // sets _frequency_mhz by the current band and channel
    }

    _defaults_set = true;
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "VTX Defaults set, freq: %u conf vs %u vtx", _frequency_mhz.get(), _current_frequency);

    announce_vtx_settings();

    return true;
}

void AP_VideoTX::announce_vtx_settings() const
{
    // Output a friendly message so the user knows the VTX has been detected
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VTX: %s%d %dMHz, %dmW #%d",
        band(_band.get()).name, _channel.get() + 1, _frequency_mhz.get(),
        has_option(VideoOptions::VTX_PITMODE) ? 0 : _power_mw.get(), _current_power);
}

// change the video power based on switch input
// 6-pos range is in the middle of the available range
void AP_VideoTX::change_power(int8_t position)
{
    if (!_enabled || position < 0 || position >= _num_active_levels)
        return;

    // first find out how many possible levels there are
    uint8_t num_active_levels = 0;
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++)
        if (_power_levels[i].active != PowerActive::Inactive && _power_levels[i].mw <= _max_power_mw)
            ++num_active_levels;
    if(num_active_levels > _num_active_levels) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "The actual number of power levels (%u) > _num_active_levels (%u)", num_active_levels, uint8_t(_num_active_levels));
        // num_active_levels = _num_active_levels;
    }
    // Iterate through to find the level
    // The level mapping is necessary only when num_active_levels != positions (6 for 6pos switch)
    // const uint16_t level = constrain_int16(roundf((num_active_levels * (position + 1) / 6.f) - 1), 0, num_active_levels - 1);
    const uint16_t level = round_div<uint8_t>(num_active_levels * (position + 1), _num_active_levels) - 1;
    // const uint16_t level = position;
    debug("looking for pos %d power level %d from %d", position, level, num_active_levels);
    uint16_t power = 0;
    for (uint8_t i = 0, j = 0; i < num_active_levels; ++i, ++j) {
        while (j < VTX_MAX_POWER_LEVELS-1 && _power_levels[j].active == PowerActive::Inactive)
            ++j;
        if (i == level) {
            if(j >= VTX_MAX_POWER_LEVELS) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "The actual number of active levels (%u) < num_active_levels (%u)", i, num_active_levels);
                return;
            }
            power = _power_levels[j].mw;
            debug("selected power %dmw", power);
            break;
        }
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Setting VTX pwr: %u mw #%u", power, position);
    if (power == 0) {
        // NOTE: We might intentionally want to turn off VTX to reduce RF emissions temporary
        // if (!hal.util->get_soft_armed())    // Don't allow pitmode to be entered if already armed
            set_configured_options(get_configured_options() | uint8_t(VideoOptions::VTX_PITMODE));
    } else {
        if (has_option(VideoOptions::VTX_PITMODE))
            set_configured_options(get_configured_options() & ~uint8_t(VideoOptions::VTX_PITMODE));
        set_configured_power_mw(power);
    }
}

bool AP_VideoTX::band_valid(uint8_t band) const
{
    // VTX Band E [0, MAX_BANDS)
    // assert(band < AP_VideoTX::VideoBand::MAX_BANDS && "The band value is out of range");
    if (band >= bands_num()) {  // AP_VideoTX::VideoBand::MAX_BANDS
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Out of range, omitting band: %u (>= %u)", band, bands_num());
        return false;
    }
    return true;
}

bool AP_VideoTX::channel_valid(uint8_t channel) const
{
    // Channel: 0..7
    if (channel >= 8) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Out of range, omitting channel: %u (>= 8)", channel);
        return false;
    }
    return true;
}

namespace AP {
    AP_VideoTX& vtx() {
        return *AP_VideoTX::get_singleton();
    }
};

#endif
