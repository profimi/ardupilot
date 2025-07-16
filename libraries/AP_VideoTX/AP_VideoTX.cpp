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
    // Range: (MAX_BANDS - 1)*10 + (VTX_MAX_CHANNELS - 1) = 167 < 317 ((2^5-1)*10 + 2^3-1)
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
    // @Description: How many proper power levels has been configured, < VTX_MAX_ADJUSTABLE_POWER_LEVELS = 6
    // @Range: 0 VTX_MAX_ADJUSTABLE_POWER_LEVELS
    AP_GROUPINFO("POW_LEVELS", 15, AP_VideoTX, _num_active_levels, 6),

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

const char * AP_VideoTX::band_names[] = {"A","B","E","F","R","L",
    "AKK5_F", // "1G3_A",
    "AKK5_L", // "1G3_B",
    "X","3G3_A", // "3G3_B",
    "AKK5_U",
    "P", "l","U","O","C" // "D1_S", "AKK5_U"
};

// CAUTION: MAX_BANDS * VTX_MAX_CHANNELS <= 256 (1 byte), otherwise libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp, update_vtx_params()
// and other functions should be updated
// Note: SmartAudio v2.0 uses internal bands of a particular VTX unlike IRC Tramp
// ATTENTION: Must be synced with the enums: VideoBand, band_names
static_assert(AP_VideoTX::MAX_BANDS * VTX_MAX_CHANNELS <= 256, "VTX channel operations, including telemetry should be adapted for 2-byte absolute channel.");
const uint16_t AP_VideoTX::VIDEO_CHANNELS[AP_VideoTX::MAX_BANDS][VTX_MAX_CHANNELS] =
{
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, /* 0 Band A, o; AKK O */
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, /* 1 Band B, x; AKK H */
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, /* 2 Band E; AKK T */
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, /* 3 Airwave,FATSHARK, F; AKK n */
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, /* 4 Race, R */
    { 5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621}, /* 5 LO Race, L; AKK b */
    // { 5621, 5584, 5547, 5510, 5473, 5436, 5399, 5362}, /* 5 Ardupilot's original LO Race, L */
    { 5129, 5159, 5189, 5219, 5249, 5279, 5309, 5339}, /* 6 AKK F */
    // { 1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360}, /* 6 Band 1G3_A */
    { 4900, 4940, 4921, 4958, 4995, 5032, 5069, 5099}, /* 7 AKK L */
    // { 1080, 1120, 1160, 1200, 1258, 1280, 1320, 1360}, /* 7 Band 1G3_B */
    { 4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200}, /* 8 Band X, b; AKK r */
    { 3330, 3350, 3370, 3390, 3410, 3430, 3450, 3470}, /* 9 Band 3G3_A */
    { 5960, 5980, 6000, 6020, 6030, 6040, 6050, 6060}, /* A AKK U */
    // { 3170, 3190, 3210, 3230, 3250, 3270, 3290, 3310}, /* A Band 3G3_B */
    // Custom and additional bands
    { 5653, 5693, 5733, 5773, 5813, 5853, 5893, 5933}, /* B Band P, H */
    { 5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613}, /* C Band l of AKK, L of Fox10; AKK P */
    { 5325, 5348, 5366, 5384, 5402, 5420, 5438, 5456}, /* D Band U; AKK E */
    { 5474, 5492, 5510, 5528, 5546, 5564, 5582, 5600}, /* E Band O; AKK A */
    // { 6002, 6028, 6054, 6002, 6002, 6002, 6002, 6002}, /* F D1 Band S */
    { 6080, 6100, 5362, 5658, 5945, 6002, 6028, 6054}, /* F Band C, Custom */
};

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
    // { 0,    10,   10, 5  }, // D1; AKK8/5/3
    { 0,    25,   14, 7  }, // D1; AKK8/5/3
    { 0x11, 100,  20, 10 }, // only in SA 2.1
    { 1,    200,  23, 16 }, // AKK5
    { 0x12, 400,  26, 20 }, // only in SA 2.1
    { 2,    500,  27, 25 }, // D1; AKK5; Fxr10
    { 0x22, 600,  28, 30 },
    { 3,    800,  29, 40 },
    { 0x13, 1000, 30, 50 }, // only in SA 2.1; D1; AKK8/5/3
    { 4,    1200, 31, 52 },
    { 0x14, 1600, 32, 56 },
    { 5,    2000, 33, 60 },
    { 0x15, 2500, 34, 65 }, // D1; Fxr10
    { 0x16, 3000, 35, 70 }, // AKK8/5/3
    { 0x17, 5000, 37, 80 }, // AKK8/5 (AKK Ultra Long Range 5W TX5000ac 6060 Mhz); Fxr10
    { 0x18, 7500, 39, 85 }, // Fxr10 (Foxeer 4.9G~6G Reaper Infinity 10W)
    { 0x19, 8000, 39, 90 }, // AKK8 (AKK Ultra Long Range 8W TX5000ac 6060 Mhz)
    { 0x20, 10000, 40, 100 }, // Foxeer 4.9G~6G Reaper Infinity 10W
    { 0xFF, 0,    0,  0XFF, PowerActive::Inactive }  // slot reserved for a custom power level
};

const uint8_t VTX_MAX_POWER_LEVELS = sizeof(AP_VideoTX::_power_levels) / sizeof(AP_VideoTX::_power_levels[0]);

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
    if(_num_active_levels >= VTX_MAX_ADJUSTABLE_POWER_LEVELS)
        _num_active_levels.set_and_save(VTX_MAX_ADJUSTABLE_POWER_LEVELS);

    /*! @brief Power levels initialization
    * @param[in] pwrMax  - the number of power levels
    * @param[in] mws  - milli Watt values
    * @param[in] doEnum  - whether to reset and enumerate the level values corresponding to those power values,
    *   which is essential for SmartAudio 2.0
    */
    auto initPowerLevels = [this](uint16_t pwrMax, std::initializer_list<uint16_t>&& mws, bool doEnum=true)
    {
        _max_power_mw.set_and_save(pwrMax);
        _num_active_levels.set_and_save(mws.size());
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
    };

    // Make inactive power levels exceeding the power capacity of the target VTX
    switch (model()) {
    case Model::D1:
        initPowerLevels(2500, {25, 500, 1000, 2500});
        break;
    case Model::FXR10: {
        _max_power_mw.set_and_save(10000);
        std::initializer_list<PowerValue> pws = {
            // value, mW
            {25, 500},
            {100, 2500},
            {200, 5000},
            {400, 7500},
            {600, 10000}
        };
        std::copy(pws.begin(), pws.end(), _power_vals);
        _num_active_levels.set_and_save(5);  // ATTENTION: must be synced with the actual values of _power_vals
        validate_cpowlevs();
        break;
    }
    case Model::AKK8:
        initPowerLevels(8000, {25, 1000, 3000, 5000, 8000}, true);
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

    _current_band = _band;
    _current_channel = _channel;
    _current_frequency = _frequency_mhz;
    _current_options = _options;
    _current_enabled = _enabled;
    _initialized = true;

    return true;
}

bool AP_VideoTX::get_band_and_channel(uint16_t freq, VideoBand& band, uint8_t& channel)
{
    for (uint8_t i = 0; i < AP_VideoTX::MAX_BANDS; i++) {
        for (uint8_t j = 0; j < VTX_MAX_CHANNELS; j++) {
            if (VIDEO_CHANNELS[i][j] == freq) {
                band = VideoBand(i);
                channel = j;
                return true;
            }
        }
    }
    return false;
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
    for (; i < VTX_MAX_POWER_LEVELS && power <= _power_levels[i].dbm; ++i) {
        if (power == _power_levels[i].dbm) {
            if (_power_levels[i].active != active) {
                _power_levels[i].active = active;
                debug("%s power %ddbm", active == PowerActive::Active ? "learned" : "invalidated", power);
            }
            return i;
        }
    }
    // Insert new power levels if necessary, moving the previous value to the reserved custom slot
    if(i < VTX_MAX_POWER_LEVELS) {
        if(i < VTX_MAX_POWER_LEVELS-1)
            _power_levels[VTX_MAX_POWER_LEVELS-1] = _power_levels[i];
        _power_levels[i].dbm = power;
        _power_levels[i].level = 0xFF;
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
        for (; i < VTX_MAX_ADJUSTABLE_POWER_LEVELS && _power_vals[i].val <= val; ++i)
            if (val == _power_vals[i].val)
                return _power_vals[i].mw;
        if (i > 0 && _power_vals[i].mw - val > val - _power_vals[i-1].mw)
            --i;
        return _power_vals[i].mw;
    };

    if (cmw(power) == _power_levels[_current_power].mw
    && _power_levels[_current_power].active == active)
        return;

    for (uint8_t i = 0, j = 0; i < VTX_MAX_POWER_LEVELS && j < VTX_MAX_ADJUSTABLE_POWER_LEVELS; ++i) {
        if (_power_levels[i].mw == _power_vals[j].mw) {
            if (power == _power_vals[j].val) {
                _current_power = i;
                _power_levels[i].active = active;
                debug("learned power %dmw", get_power_mw());
                break;
            } else ++j;
        }
    }
}

uint16_t AP_VideoTX::get_configured_power_val() const
{
     for(uint8_t i = 0; i < VTX_MAX_POWER_LEVELS && _power_vals[i].mw <= _power_mw; ++i)
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
    if (!_defaults_set) {
        return false;
    }
    // check pitmode
    if ((_options & uint8_t(VideoOptions::VTX_PITMODE))
    != (_current_options & uint8_t(VideoOptions::VTX_PITMODE))) {
        return true;
    }

#if HAL_CRSF_TELEM_ENABLED
    // using CRSF so unlock is not an option
    if (AP::crsf_telem() != nullptr) {
        return false;
    }
#endif
    // check unlock only
    if ((_options & uint8_t(VideoOptions::VTX_UNLOCKED)) != 0
    && (_current_options & uint8_t(VideoOptions::VTX_UNLOCKED)) == 0) {
        return true;
    }

    // ignore everything else
    return false;
}

void AP_VideoTX::set_preset(uint8_t preset_no)
{
    // assert(preset_no < sizeof _preset && "preset_no is out of range");
    if(preset_no >= sizeof _preset) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Out of range, omitting: preset_no = %u (>= %u)", preset_no, sizeof _preset);
        return;
    }
    // Note: heximal instead of the decimal digit system is used to cover up to 16 bands
    set_band(_preset[preset_no] / 10);
    set_channel(_preset[preset_no] % 10);
}

bool AP_VideoTX::update_power() const {
    if (!_defaults_set || _power_mw == get_power_mw() || get_pitmode()) {
        return false;
    }
    // check that the requested power is actually allowed
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS && _power_mw >= _power_levels[i].mw; i++) {
        if (_power_mw == _power_levels[i].mw
            && _power_levels[i].active != PowerActive::Inactive) {
            return true;
        }
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
void AP_VideoTX::update_configured_channel_and_band()
{
    VideoBand band;
    uint8_t channel;
    if (get_band_and_channel(_frequency_mhz, band, channel)) {
        _band.set_and_save(band);
        _channel.set_and_save(channel);
    } else {
        update_configured_frequency();
    }
}

// set the current configured values if not currently set in storage
// this is necessary so that the current settings can be seen
bool AP_VideoTX::set_defaults()
{
    if (_defaults_set) {
        return false;
    }

    // check that our current view of frequency matches band/channel
    // if not then force one to be correct
    uint16_t calced_freq = get_frequency_mhz(_current_band, _current_channel);
    if (_current_frequency != calced_freq) {
        if (_current_frequency > 0) {
            VideoBand band;
            uint8_t channel;
            if (get_band_and_channel(_current_frequency, band, channel)) {
                _current_band = band;
                _current_channel = channel;
            } else {
                _current_frequency = calced_freq;
            }
        } else {
            _current_frequency = calced_freq;
        }
    }

    if (!_options.configured()) {
        _options.set_and_save(_current_options);
    }
    if (!_channel.configured()) {
        _channel.set_and_save(_current_channel);
    }
    if (!_band.configured()) {
        _band.set_and_save(_current_band);
    }
    if (!_power_mw.configured()) {
        _power_mw.set_and_save(get_power_mw());
    }
    if (!_frequency_mhz.configured()) {
        _frequency_mhz.set_and_save(_current_frequency);
    }

    // Now check that the user didn't screw up by selecting incompatible options
    if (_frequency_mhz != get_frequency_mhz(_band, _channel)) {
        if (_frequency_mhz > 0) {
            update_configured_channel_and_band();
        } else {
            update_configured_frequency();
        }
    }

    _defaults_set = true;

    announce_vtx_settings();

    return true;
}

void AP_VideoTX::announce_vtx_settings() const
{
    // Output a friendly message so the user knows the VTX has been detected
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VTX: %s%d %dMHz, PWR: %dmW (%dmW #%d)",
        band_names[_band.get()], _channel.get() + 1, _frequency_mhz.get(),
        has_option(VideoOptions::VTX_PITMODE) ? 0 : _power_mw.get(), get_power_mw(), _current_power);
}

// change the video power based on switch input
// 6-pos range is in the middle of the available range
void AP_VideoTX::change_power(int8_t position)
{
    if (!_enabled || position < 0 || position >= _num_active_levels)
        return;

    // first find out how many possible levels there are
    uint8_t num_active_levels = 0;
    for (uint8_t i = 0; i < VTX_MAX_POWER_LEVELS; i++) {
        if (_power_levels[i].active != PowerActive::Inactive && _power_levels[i].mw <= _max_power_mw) {
            num_active_levels++;
        }
    }
    // iterate through to find the level
    uint16_t level = constrain_int16(roundf((num_active_levels * (position + 1) / float(_num_active_levels)) - 1), 0, num_active_levels - 1);
    debug("looking for pos %d power level %d from %d", position, level, num_active_levels);
    uint16_t power = 0;
    for (uint8_t i = 0, j = 0; i < num_active_levels; ++i, ++j) {
        while (j < VTX_MAX_POWER_LEVELS-1 && _power_levels[j].active == PowerActive::Inactive)
            ++j;
        if (i == level) {
            if(j >= VTX_MAX_POWER_LEVELS) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "The number of actual active levels (%u) < num_active_levels (%u)", i, num_active_levels);
                return;
            }
            power = _power_levels[j].mw;
            debug("selected power %dmw", power);
            break;
        }
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Setting VTX power to %u mw (#%u)", power, position);
    if (power == 0) {
        // NOTE: We might intentionally want to turn off VTX to reduce/hide our radio profile unil moving to some further location
        // if (!hal.util->get_soft_armed())    // don't allow pitmode to be entered if already armed
            set_configured_options(get_configured_options() | uint8_t(VideoOptions::VTX_PITMODE));
    } else {
        if (has_option(VideoOptions::VTX_PITMODE)) {
            set_configured_options(get_configured_options() & ~uint8_t(VideoOptions::VTX_PITMODE));
        }
        set_configured_power_mw(power);
    }
}

bool AP_VideoTX::band_valid(uint8_t band) const
{
    // VTX Band E [0, MAX_BANDS)
    // assert(band < AP_VideoTX::VideoBand::MAX_BANDS && "The band value is out of range");
    if (band >= AP_VideoTX::VideoBand::MAX_BANDS) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Out of range, omitting: band = %u (>= %u)", band, AP_VideoTX::VideoBand::MAX_BANDS);
        return false;
    }
    return true;
}

bool AP_VideoTX::channel_valid(uint8_t channel) const
{
    // Channel: 0..7
    if (channel >= 8) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Out of range, omitting: channel = %u (>= 8)", channel);
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
