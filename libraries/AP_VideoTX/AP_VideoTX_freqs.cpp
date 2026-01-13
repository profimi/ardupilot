#include "AP_VideoTX_freqs.h"

using namespace VTX;

Band band_A{{5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, "A"};  // 0 Band A, o; AKK O
Band band_B{{5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, "B"};  // 1 Band B, x; AKK H
Band band_E{{5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, "E"};  // 2 Band E; AKK T
Band band_F{{5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, "F"};  // 3 Airwave,FATSHARK,IRC/FS; F/I; AKK n
Band band_R{{5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, "R"};  // 4 Race, R
Band band_L{{5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621}, "L"};  // 5 LO Race, L; AKK b
Band band_M{{5658, 5678, 5717, 5737, 5835, 5855, 5894, 5914}, "M"};  // 6 M
Band band_X{{4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200}, "X"};  // 7 Band X, b; AKK r; TBS H
Band band_Y{{4991, 5045, 5097, 5128, 5329, 5435, 5477, 5590}, "Y"};  // 8 Band Y
Band band_T{{5176, 5261, 5323, 5360, 5946, 5974, 6013, 6030}, "T"};  // 9 Band T

Band band_AP_L{{5621, 5584, 5547, 5510, 5473, 5436, 5399, 5362}, "AP_L"};  // A AP_L: Ardupilot's LO = reversed standard Low Race
Band band_1G3_A{{1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360}, "1G3_A"};  // B 1G3_A
Band band_1G3_B{{1080, 1120, 1160, 1200, 1258, 1280, 1320, 1360}, "1G3_B"};  // C 1G3_B
Band band_3G3_A{{3330, 3350, 3370, 3390, 3410, 3430, 3450, 3470}, "3G3_A"};  // D Band 3G3_A
Band band_3G3_B{{3170, 3190, 3210, 3230, 3250, 3270, 3290, 3310}, "3G3_B"};  // E Band 3G3_B

// CAUTION: MAX_BANDS * BAND_CHANNELS_NUM <= 256 (1 byte), otherwise libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp, update_vtx_params()
// and other functions should be updated
// Note: SmartAudio v2.0 uses internal bands of a particular VTX unlike IRC Tramp
Band freqs_generic[]{band_A, band_B, band_E, band_F, band_R, band_L, band_M, band_X, band_Y, band_T
    // , band_AP_L, band_1G3_A, band_1G3_B, band_3G3_A, band_3G3_B
};
constexpr uint8_t MAX_BANDS_GENERIC = sizeof(freqs_generic) / sizeof(freqs_generic[0]);

static_assert(MAX_BANDS_GENERIC * BAND_CHANNELS_NUM <= 256, "VTX channel operations, including telemetry should be adapted for 2-byte absolute channel.");


void FreqMap::init(Model model, bool forceFreq)
{
    _model = model;
    _forceFreq = forceFreq;
}

uint8_t FreqMap::bands_num() const
{
    return MAX_BANDS_GENERIC;
}

const Band& FreqMap::band(uint8_t i) const
{
    return freqs_generic[i];
}

// Band* FreqMap::bands()
// {
//     return freqs_generic;
// }

uint16_t FreqMap::freq(uint8_t band, uint8_t channel) const
{
    return freqs_generic[band].channels[channel];
}
