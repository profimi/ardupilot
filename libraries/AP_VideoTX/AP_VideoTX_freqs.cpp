#include "AP_VideoTX_freqs.h"

using namespace VTX;

Band band_A{{5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, "A"};  // 0 Band A, o; AKK O
Band band_B{{5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, "B"};  // 1 Band B, x; AKK H
Band band_E{{5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, "E"};  // 2 Band E; AKK T
Band band_F{{5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, "F"};  // 3 Airwave,FATSHARK,IRC/FS; F/I; AKK n
Band band_R{{5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, "R"};  // 4 Race, R
Band band_L{{5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621}, "L"};  // 5 LO Race, L; AKK b
Band band_AKK_F{{5129, 5159, 5189, 5219, 5249, 5279, 5309, 5339}, "AKK_F"};  // 6 AKK F
Band band_AKK_L{{4900, 4940, 4921, 4958, 4995, 5032, 5069, 5099}, "AKK_L"};  // 7 AKK L
Band band_X{{4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200}, "X"};  // 8 Band X, b; AKK r
Band band_3G3_A{{3330, 3350, 3370, 3390, 3410, 3430, 3450, 3470}, "3G3_A"};  // 9 Band 3G3_A
Band band_AKK_U{{5960, 5980, 6000, 6020, 6030, 6040, 6050, 6060}, "AKK_U"};  // A Band AKK_U
Band band_P{{5653, 5693, 5733, 5773, 5813, 5853, 5893, 5933}, "P"};  // B Band P, H
Band band_l{{5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613}, "l"};  // C Band l of AKK, L of Fox10; AKK P
Band band_U{{5325, 5348, 5366, 5384, 5402, 5420, 5438, 5456}, "U"};  // D Band U; AKK E
Band band_O{{5474, 5492, 5510, 5528, 5546, 5564, 5582, 5600}, "O"};  // E Band O; AKK A
Band band_C{{6080, 6100, 5362, 5658, 5945, 6002, 6028, 6054}, "C"};  // F Band C, Custom

// TBS bands
Band band_M{{5658, 5678, 5717, 5737, 5835, 5855, 5894, 5914}, "M"};  // 6 Band M
Band band_Y{{4991, 5045, 5097, 5128, 5329, 5435, 5477, 5590}, "Y"};  // 8 Band Y
Band band_T{{5176, 5261, 5323, 5360, 5946, 5974, 6013, 6030}, "T"};  // 9 Band T
// Ardupilot bands
Band band_AP_L{{5621, 5584, 5547, 5510, 5473, 5436, 5399, 5362}, "AP_L"};  // 5 AP_L: Ardupilot's LO = reversed standard Low Race
Band band_1G3_A{{1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360}, "1G3_A"};  // 6 1G3_A
Band band_1G3_B{{1080, 1120, 1160, 1200, 1258, 1280, 1320, 1360}, "1G3_B"};  // 7 1G3_B
Band band_3G3_B{{3170, 3190, 3210, 3230, 3250, 3270, 3290, 3310}, "3G3_B"};  // A Band 3G3_B

// CAUTION: MAX_BANDS * BAND_CHANNELS_NUM <= 256 (1 byte), otherwise libraries/AP_RCTelemetry/AP_CRSF_Telem.cpp, update_vtx_params()
// and other functions should be updated
// Note: SmartAudio v2.0 uses internal bands of a particular VTX unlike IRC Tramp
Band freqs_generic[]{band_A, band_B, band_E, band_F, band_R, band_L, band_AKK_F, band_AKK_L, band_X
    , band_3G3_A, band_AKK_U, band_P, band_l, band_U, band_O, band_C};
constexpr uint8_t MAX_BANDS_GENERIC = sizeof(freqs_generic) / sizeof(freqs_generic[0]);

Band freqs_akk8[]{band_O, band_L, band_U, band_AKK_F, band_X, band_l
    , band_AKK_L, band_AKK_U, band_A, band_B, band_E, band_F};
constexpr uint8_t MAX_BANDS_AKK8 = sizeof(freqs_akk8) / sizeof(freqs_akk8[0]);

// Rapidfire IRC:  IRC/FatShark, RaceBand, LowRace, Band A, B, E, Favorites
Band freqs_rfire[]{band_F, band_R, band_L, band_A, band_B, band_E, band_C};
constexpr uint8_t MAX_BANDS_RFIRE = sizeof(freqs_rfire) / sizeof(freqs_rfire[0]);

static_assert(MAX_BANDS_GENERIC >= MAX_BANDS_AKK8 && MAX_BANDS_GENERIC >= MAX_BANDS_RFIRE, "Unexpected size of bands in the frequency mappings");
static_assert(MAX_BANDS_GENERIC * BAND_CHANNELS_NUM <= 256, "VTX channel operations, including telemetry should be adapted for 2-byte absolute channel.");


void FreqMap::init(Model model, bool forceFreq)
{
    _model = model;
    _forceFreq = forceFreq;
}

uint8_t FreqMap::bands_num() const
{
    return !_forceFreq && _model == Model::AKK8 ? MAX_BANDS_AKK8 : MAX_BANDS_GENERIC;
}

const Band& FreqMap::band(uint8_t i) const
{
    return !_forceFreq && _model == Model::AKK8 ? freqs_akk8[i] : freqs_generic[i];
}

// Band* FreqMap::bands()
// {
//     return !_forceFreq && vtx == Model::AKK8 ? freqs_akk8 : freqs_generic;
// }

uint16_t FreqMap::freq(uint8_t band, uint8_t channel) const
{
    return !_forceFreq && _model == Model::AKK8 ? freqs_akk8[band].channels[channel] : freqs_generic[band].channels[channel];
}
