#pragma once

#include <cstdint>

namespace VTX {
constexpr uint8_t BAND_CHANNELS_NUM = 8;  // The [maximal] number of channels in bands

// VTX Model
enum class Model: uint8_t {
    GENERIC = 0,
    D1 = 1,  // D1 accepts power values in DBM for both IRC Tramp and SmartAudio 2.1
    FXR10 = 2,  // Foxeer 4.9G~6G Reaper Infinity 10W 80CH VTx; accepts old IRC Tramp mW values for another actual power levels: 25 -> 500mw, 100 -> 2.5W, 200 -> 5W, 400 -> 7.5W, 600 -> 10W
    // AKK5 = 3,  // Accepts IRC Tramp values in levels: 0 .. 4; AKK Ultra Long Range 5W: 25/200/500/1000/3000/5000mW
    AKK8 = 3,  // AKK TX8000AC Ultra Long Range 8W: 20/1000/3000/5000/8000 mW
    CUSTOM = 9  // 6 custom power values
};

struct Band {
    uint16_t channels[BAND_CHANNELS_NUM];
    const char *name;
};

// Band band_A, band_B, band_E, band_F, band_R, band_L, band_AKK_F, band_AKK_L, band_X, band_3G3_A
//     , band_AKK_U, band_P, band_l, band_U, band_O, band_C
//     , band_AP_L, band_1G3_A, band_1G3_B, band_3G3_B, band_D1_S;

class FreqMap {
private:
    Model _model;   /// VTX model
    bool _forceFreq;  /// Force VTX switching by the specified frequency rather than index in the frequency mapping table of the VTX; Essential for SmartAudio 2.0
public:
    // FreqMap(Model model, bool forceFreq);
    // virtual ~FreqMap()  {}
    void init(Model model, bool forceFreq);

    uint8_t bands_num() const;  /// The number of bands in the current VTX model
    const Band& band(uint8_t i) const;  /// Specific band of the current VTX model
    uint16_t freq(uint8_t band, uint8_t channel) const;  /// Current VTX frequency map
};
}  // VTX
