#ifndef FCUEFIS_AIRCRAFT_PROFILE_H
#define FCUEFIS_AIRCRAFT_PROFILE_H

#include <string>
#include <vector>
#include <functional>
#include <map>
#include <unordered_map>
#include <cstdint>

// 7-segment display character representations
// Bit mapping: 0x80=Top, 0x40=Upper Right, 0x20=Lower Right, 0x10=Bottom, 0x08=Upper Left, 0x04=Middle, 0x02=Lower Left, 0x01=Dot
static const std::unordered_map<char, uint8_t> SEGMENT_REPRESENTATIONS = {
    {'0', 0xFA}, {'1', 0x60}, {'2', 0xD6}, {'3', 0xF4}, {'4', 0x6C},
    {'5', 0xBC}, {'6', 0xBE}, {'7', 0xE0}, {'8', 0xFE}, {'9', 0xFC},
    {'A', 0xEE}, {'B', 0xFE}, {'C', 0x9A}, {'D', 0x76}, {'E', 0x9E},
    {'F', 0x8E}, {'G', 0xBE}, {'H', 0x6E}, {'I', 0x60}, {'J', 0x70},
    {'K', 0x0E}, {'L', 0x1A}, {'M', 0xA6}, {'N', 0x26}, {'O', 0xFA},
    {'P', 0xCE}, {'Q', 0xEC}, {'R', 0x06}, {'S', 0xBC}, {'T', 0x1E},
    {'U', 0x7A}, {'V', 0x32}, {'W', 0x58}, {'X', 0x6E}, {'Y', 0x7C},
    {'Z', 0xD6}, {'-', 0x04}, {'#', 0x36}, {'/', 0x60}, {'\\', 0xA0}, {' ', 0x00}
};

enum class DisplayByteIndex : int {
    H0 = 0, H3 = 1, A0 = 2, A1 = 3, A2 = 4, A3 = 5, A4 = 6, A5 = 7,
    V2 = 8, V3 = 9, V0 = 10, V1 = 11, S1 = 12,
    EFISR_B0 = 13, EFISR_B2 = 14, EFISL_B0 = 15, EFISL_B2 = 16
};

struct DisplayFlag {
    std::string name;
    DisplayByteIndex byteIndex;
    uint8_t mask;
    bool defaultValue;
    
    DisplayFlag(const std::string& n, DisplayByteIndex idx, uint8_t m, bool def = false)
        : name(n), byteIndex(idx), mask(m), defaultValue(def) {}
};

struct FCUEfisButtonDef {
    int id;
    std::string name;
    std::string dataref;
    int value = -1;  // Optional: for buttons that set specific values (-1 means use command)
};
enum class FCUEfisLed : int {
    // FCU LEDs
    BACKLIGHT = 0,
    SCREEN_BACKLIGHT = 1,
    LOC_GREEN = 3,
    AP1_GREEN = 5,
    AP2_GREEN = 7,
    ATHR_GREEN = 9,
    EXPED_GREEN = 11,
    APPR_GREEN = 13,
    FLAG_GREEN = 17,
    EXPED_YELLOW = 30,
    
    // EFIS Right LEDs (100-199)
    EFISR_BACKLIGHT = 100,
    EFISR_SCREEN_BACKLIGHT = 101,
    EFISR_FLAG_GREEN = 102,
    EFISR_FD_GREEN = 103,
    EFISR_LS_GREEN = 104,
    EFISR_CSTR_GREEN = 105,
    EFISR_WPT_GREEN = 106,
    EFISR_VORD_GREEN = 107,
    EFISR_NDB_GREEN = 108,
    EFISR_ARPT_GREEN = 109,
    
    // EFIS Left LEDs (200-299)
    EFISL_BACKLIGHT = 200,
    EFISL_SCREEN_BACKLIGHT = 201,
    EFISL_FLAG_GREEN = 202,
    EFISL_FD_GREEN = 203,
    EFISL_LS_GREEN = 204,
    EFISL_CSTR_GREEN = 205,
    EFISL_WPT_GREEN = 206,
    EFISL_VORD_GREEN = 207,
    EFISL_NDB_GREEN = 208,
    EFISL_ARPT_GREEN = 209
};

struct FCUDisplayData {
    std::string speed;
    std::string heading;
    std::string altitude;
    std::string verticalSpeed;
    std::string efisRBaro;
    std::string efisLBaro;
    
    // Display flags
    bool spdMach = false;
    bool hdgTrk = false;
    bool altManaged = false;
    bool spdManaged = false;
    bool hdgManaged = false;
    bool vsMode = false;
    bool fpaMode = false;
    bool efisRQnh = false;
    bool efisLQnh = false;
    bool efisRHpaDec = false;
    bool efisLHpaDec = false;
    
    // Additional display flags for proper 7-segment display
    bool latMode = false;
    bool altIndication = true;
    bool vsHorizontalLine = true;
    bool vsVerticalLine = false;
    bool lvlChange = true;
    bool lvlChangeLeft = true;
    bool lvlChangeRight = true;
    bool vsIndication = false;
    bool fpaIndication = false;
    bool fpaComma = false;
    bool machComma = false;
    bool efisRQfe = false;
    bool efisLQfe = false;
};

class FCUEfisAircraftProfile {
public:
    FCUEfisAircraftProfile() {};
    virtual ~FCUEfisAircraftProfile() = default;
    
    std::function<void(FCUEfisLed led, unsigned char brightness)> ledBrightnessCallback = nullptr;

    virtual const std::vector<std::string>& displayDatarefs() const = 0;
    virtual const std::vector<FCUEfisButtonDef>& buttonDefs() const = 0;
    virtual void updateDisplayData(FCUDisplayData& displayData, const std::map<std::string, std::string>& cachedDatarefValues) = 0;
    virtual bool hasEfisRight() const = 0;
    virtual bool hasEfisLeft() const = 0;
};

#endif
