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
#pragma once

#include "AP_VideoTX_config.h"
#include <array>

#if AP_VIDEOTX_ENABLED

#include <AP_Param/AP_Param.h>

constexpr uint8_t VTX_MAX_CHANNELS = 8;
constexpr uint8_t VTX_MAX_ADJUSTABLE_POWER_LEVELS = 6;  // <= 7, typically 5-6
extern const uint8_t VTX_MAX_POWER_LEVELS;  // = 19;

class AP_VideoTX {
public:
    AP_VideoTX();
    ~AP_VideoTX();

    // VTX Model
    enum class Model: uint8_t {
        GENERIC = 0,
        D1 = 1,  // D1 accepts power values in DBM for both IRC Tramp and SmartAudio 2.1
        FXR10 = 2,  // Foxeer 4.9G~6G Reaper Infinity 10W 80CH VTx; accepts old IRC Tramp mW values for another actual power levels: 25 -> 500mw, 100 -> 2.5W, 200 -> 5W, 400 -> 7.5W, 600 -> 10W
        // AKK5 = 3,  // Accepts IRC Tramp values in levels: 0 .. 4; AKK Ultra Long Range 5W: 25/200/500/1000/3000/5000mW
        AKK8 = 3,  // AKK TX8000AC Ultra Long Range 8W: 20/1000/3000/5000/8000 mW
        CUSTOM = 9  // 6 custom power values
    };

    /* Do not allow copies */
    CLASS_NO_COPY(AP_VideoTX);

    // Perform required initialisation because param.set_and_save() can't be used in the constructor
    bool init();

    // run any required updates
    void update();

    static AP_VideoTX *get_singleton(void) {
        return singleton;
    }
    static const struct AP_Param::GroupInfo var_info[];

    enum class VideoOptions {
        VTX_PITMODE           = (1 << 0),
        VTX_PITMODE_UNTIL_ARM = (1 << 1),
        VTX_PITMODE_ON_DISARM = (1 << 2),
        VTX_UNLOCKED          = (1 << 3),
        VTX_PULLDOWN          = (1 << 4),
        VTX_SA_ONE_STOP_BIT   = (1 << 5),
        VTX_SA_IGNORE_CRC     = (1 << 6),
        VTX_CRSF_IGNORE_STAT  = (1 << 7),
    };

    static const char *band_names[];

    // Note: SmartAudi v2.0 uses internal bands of a particular VTX unlike IRC Tramp
    enum VideoBand {
        BAND_A,
        BAND_o = BAND_A,
        BAND_B,
        BAND_x = BAND_B,
        BAND_E,
        FATSHARK,  // Airwave
        BAND_F = FATSHARK,
        RACEBAND,
        BAND_R = RACEBAND,
        LOW_RACEBAND,
        BAND_L = LOW_RACEBAND,
        // BAND_1G3_A,
        BAND_AKK5_F,
        // BAND_1G3_B,
        BAND_AKK5_L,
        BAND_X,
        BAND_b = BAND_X,
        BAND_3G3_A,
        BAND_3G3_B,
        // Custom bands
        BAND_P,
        BAND_H = BAND_P,
        BAND_l,
        BAND_U,
        BAND_O,
        // BAND_D1_S, BAND_AKK5_U
        BAND_C,
        MAX_BANDS
        // ATTENTION: SmartAudio channel setting and AP_CRSF_Telem.cpp have been
        // reimplemented to consider multi-byte frequency tables if MAX_BANDS > 16
    };

    enum class PowerActive {
        Unknown,
        Active,
        Inactive
    };

    enum VTXType {
        CRSF = 1U<<0,
        SmartAudio = 1U<<1,
        Tramp = 1U<<2
    };

    struct PowerLevel {
        uint8_t level;
        uint16_t mw;
        uint8_t dbm;
        uint8_t dac; // SmartAudio v1 dac value
        PowerActive active;
    };

    struct PowerValue {
        uint16_t val;  // VTX value
        uint16_t mw;  // Actual power in mW
    };

    // Mapping of power level to milli watt to dbm
    static PowerLevel _power_levels[];  // VTX_MAX_POWER_LEVELS items

    PowerValue _power_vals[VTX_MAX_ADJUSTABLE_POWER_LEVELS];  // Custom or specialized power values if necessary

    static const uint16_t VIDEO_CHANNELS[MAX_BANDS][VTX_MAX_CHANNELS];

    static uint16_t get_frequency_mhz(uint8_t band, uint8_t channel) { return VIDEO_CHANNELS[band][channel]; }
    static bool get_band_and_channel(uint16_t freq, VideoBand& band, uint8_t& channel);

    void set_frequency_mhz(uint16_t freq) { _current_frequency = freq; }
    void set_configured_frequency_mhz(uint16_t freq) { _frequency_mhz.set_and_save_ifchanged(freq); }
    uint16_t get_frequency_mhz() const { return _current_frequency; }
    uint16_t get_configured_frequency_mhz() const { return _frequency_mhz; }
    bool update_frequency() const { return _defaults_set && _frequency_mhz != _current_frequency; }
    void update_configured_frequency();
    // get / set power level
    void set_power_mw(uint16_t power);
    void set_power_level(uint8_t level, PowerActive active=PowerActive::Active);

    /*! @brief Set the power in dBm and update power levels
    * 
    * @param[in] power  - power in dBm
    * @param[in] active  - active state of the power level
    */
    void set_power_dbm(uint8_t power, PowerActive active=PowerActive::Active);
    void set_power_dac(uint16_t power, PowerActive active=PowerActive::Active);
    // add a new dbm setting to those supported, i is the starting index
    uint8_t update_power_dbm(uint8_t power, PowerActive active=PowerActive::Active, uint8_t i=0);
    void update_all_power_dbm(uint8_t nlevels, const uint8_t levels[]);
    void set_configured_power_mw(uint16_t power);

    //! Handle custom power value tables, considering power levels enumeration
    void validate_cpowlevs();
    void set_power_val(uint16_t power, PowerActive active=PowerActive::Active);
    uint16_t get_configured_power_val() const;

    uint16_t get_configured_power_mw() const { return _power_mw; }
    uint16_t get_power_mw() const { return _power_levels[_current_power].mw; }

    // get the power in dbm, rounding appropriately
    uint8_t get_configured_power_dbm() const {
        return _power_levels[find_current_power()].dbm;
    }
    // get the power "level"
    uint8_t get_configured_power_level() const {
        return _power_levels[find_current_power()].level & 0xF;
    }
    // get the power "dac"
    uint8_t get_configured_power_dac() const {
        return _power_levels[find_current_power()].dac;
    }

    bool update_power() const;
    // change the video power based on switch input
    void change_power(int8_t position);
    // Validate band and channel
    bool band_valid(uint8_t band) const;
    bool channel_valid(uint8_t channel) const;
    // get / set the frequency band
    void set_band(uint8_t band) { if(band_valid(band)) _current_band = band; }
    void set_configured_band(uint8_t band) { if(band_valid(band)) _band.set_and_save_ifchanged(band); }
    uint8_t get_configured_band() const { return _band; }
    uint8_t get_band() const { return _current_band; }
    bool update_band() const { return _defaults_set && _band != _current_band; }
    // get / set the frequency channel
    void set_channel(uint8_t channel) { if(channel_valid(channel)) _current_channel = channel; }
    void set_configured_channel(uint8_t channel) { if(channel_valid(channel)) _channel.set_and_save_ifchanged(channel); }
    uint8_t get_configured_channel() const { return _channel; }
    uint8_t get_channel() const { return _current_channel; }
    bool update_channel() const { return _defaults_set && _channel != _current_channel; }
    void update_configured_channel_and_band();
    // get / set vtx option
    void set_options(uint16_t options) { _current_options = options; }
    void set_configured_options(uint16_t options) { _options.set_and_save_ifchanged(options); }
    uint16_t get_configured_options() const { return _options; }
    uint16_t get_options() const { return _current_options; }
    bool has_option(VideoOptions option) const { return _options.get() & uint16_t(option); }
    bool get_configured_pitmode() const { return _options.get() & uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE); }
    bool get_pitmode() const { return _current_options & uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE); }
    bool update_options() const;
    // get / set whether the vtx is enabled
    void set_enabled(bool enabled);
    bool get_enabled() const { return _enabled; }
    bool update_enabled() const { return _defaults_set && _enabled != _current_enabled; }

    void set_preset(uint8_t preset_no);
    Model model() const  { return static_cast<Model>(static_cast<uint8_t>(_model)); }
    uint16_t power_at_lev(uint8_t lev, uint8_t beg=0) const;

    // have the parameters been updated
    bool have_params_changed() const;
    // set configured defaults from current settings, return true if defaults were set
    bool set_defaults();
    // display the current VTX settings in the GCS
    void announce_vtx_settings() const;
    // force the current values to reflect the configured values
    void set_power_is_current();
    void set_freq_is_current();
    void set_options_are_current() {  _current_options = _options; }

    void set_configuration_finished(bool configuration_finished) { _configuration_finished = configuration_finished; }
    bool is_configuration_finished() { return _configuration_finished; }

    // manage VTX backends
    bool is_provider_enabled(VTXType type) const { return (_types & type) != 0; }
    void set_provider_enabled(VTXType type) { _types |= type; }

    static AP_VideoTX *singleton;

private:
    uint8_t find_current_power() const;
    // channel frequency
    AP_Int16 _frequency_mhz;
    uint16_t _current_frequency;

    // power output in mw
    AP_Int16 _power_mw;
    uint16_t _current_power;
    AP_Int16 _max_power_mw;

    // frequency band
    AP_Int8 _band;
    uint16_t _current_band;

    // frequency channel
    AP_Int8 _channel;
    uint8_t _current_channel;

    // vtx options
    AP_Int16 _options;
    uint16_t _current_options;

    AP_Int8 _enabled;
    bool _current_enabled;

    // Preset block:  BBC (band 0..15 and channel 0..7)
    AP_Int16  _preset[6];

    // VTX model
    AP_Int8  _model;

    // The number of active power levels of VTX
    AP_Int8 _num_active_levels;

    // Custom VTX values and labels (mW)
    AP_Int16 _cvals[VTX_MAX_ADJUSTABLE_POWER_LEVELS];
    AP_Int16 _cmws[VTX_MAX_ADJUSTABLE_POWER_LEVELS];

    bool _initialized;
    // when defaults have been configured
    bool _defaults_set;
    // true when configuration have been applied successfully to the VTX
    bool _configuration_finished;

    // types of VTX providers
    uint8_t _types;
};

namespace AP {
    AP_VideoTX& vtx();
};

#endif  // AP_VIDEOTX_ENABLED
