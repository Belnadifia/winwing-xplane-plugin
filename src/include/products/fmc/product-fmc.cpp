// ProductFMC.cpp
#include "product-fmc.h"
#include "dataref.h"
#include "appstate.h"
#include "config.h"
#include "profiles/ff777-fmc-profile.h"
#include "profiles/ixeg733-fmc-profile.h"
#include "profiles/laminar-airbus-fmc-profile.h"
#include "profiles/ssg748-fmc-profile.h"
#include "profiles/toliss-fmc-profile.h"
#include "profiles/zibo-fmc-profile.h"
#include "profiles/xcrafts-fmc-profile.h"
#include <chrono>
#include <thread>
#include <atomic>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <XPLMProcessing.h>

ProductFMC::ProductFMC(HIDDeviceHandle hidDevice, uint16_t vendorId, uint16_t productId, std::string vendorName, std::string productName, FMCHardwareType hardwareType, unsigned char identifierByte) : USBDevice(hidDevice, vendorId, productId, vendorName, productName), hardwareType(hardwareType), identifierByte(identifierByte) {
    profile = nullptr;
    page = std::vector<std::vector<char>>(ProductFMC::PageLines, std::vector<char>(ProductFMC::PageBytesPerLine, ' '));
    lastUpdateCycle = 0;
    lastButtonStateLo = 0;
    lastButtonStateHi = 0;
    pressedButtonIndices = {};
    fontUpdatingEnabled = true;
    
    connect();
}

ProductFMC::~ProductFMC() {
    disconnect();
}

void ProductFMC::setProfileForCurrentAircraft() {
    allowFontUpdateOnce();

    if (TolissFMCProfile::IsEligible()) {
        debug("Using Toliss profile for %s.\n", classIdentifier());
        qClear();
        profile = new TolissFMCProfile(this);
        profileReady = true;
    }
    else if (LaminarFMCProfile::IsEligible()) {
        debug("Using Laminar profile for %s.\n", classIdentifier());
        qClear();
        profile = new LaminarFMCProfile(this);
        profileReady = true;
    }
    else if (XCraftsFMCProfile::IsEligible()) {
        debug("Using X-Crafts profile for %s.\n", classIdentifier());
        qClear();
        profile = new XCraftsFMCProfile(this);
        profileReady = true;
    }
    else if (ZiboFMCProfile::IsEligible()) {
        debug("Using Zibo PFP profile for %s.\n", classIdentifier());
        qClear();
        profile = new ZiboFMCProfile(this);
        profileReady = true;
    }
    else if (FlightFactor777FMCProfile::IsEligible()) {
        debug("Using FlightFactor 777 PFP profile for %s.\n", classIdentifier());
        qClear();
        profile = new FlightFactor777FMCProfile(this);
        profileReady = true;
    }
    else if (SSG748FMCProfile::IsEligible()) {
        debug("Using SSG 748 PFP profile for %s.\n", classIdentifier());
        qClear();
        profile = new SSG748FMCProfile(this);
        profileReady = true;
    }
    else if (IXEG733FMCProfile::IsEligible()) {
        debug("Using IXEG 733 PFP profile for %s.\n", classIdentifier());
        qClear();
        profile = new IXEG733FMCProfile(this);
        profileReady = true;
    }
}

const char* ProductFMC::classIdentifier() {
    if (hardwareType == FMCHardwareType::HARDWARE_MCDU) {
        return "Product FMC (MCDU)";
    }
    else if (hardwareType == FMCHardwareType::HARDWARE_PFP3N) {
        return "Product FMC (PFP3N)";
    }
    else if (hardwareType == FMCHardwareType::HARDWARE_PFP4) {
        return "Product FMC (PFP4)";
    }
    else if (hardwareType == FMCHardwareType::HARDWARE_PFP7) {
        return "Product FMC (PFP7)";
    }
    
    return "Product FMC (unknown hardware)";
}

bool ProductFMC::connect() {
    if (USBDevice::connect()) {
        uint8_t col_bg[] = {0x00, 0x00, 0x00};
        
        writeData({0xf0, 0x0, 0x1, 0x38, identifierByte, 0xbb, 0x0, 0x0, 0x1e, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x18, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0x8, 0x0, 0x0, 0x0, 0x34, 0x0, 0x18, 0x0, 0xe, 0x0, 0x18, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.97}");
        writeData({0xf0, 0x0, 0x2, 0x38, 0x0, 0x0, 0x0, 0x1, 0x0, 0x5, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0xc4, 0x24, 0xa, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x1, 0x0, 0x6, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.98}");
        writeData({0xf0, 0x0, 0x3, 0x38, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0, 0xff, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0xa5, 0xff, 0xff, 0x5, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.99}");
        writeData({0xf0, 0x0, 0x4, 0x38, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0xff, 0xff, 0xff, 0xff, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0xff, 0xff, 0x0, 0xff, 0x7, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.100}");
        writeData({0xf0, 0x0, 0x5, 0x38, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x3d, 0xff, 0x0, 0xff, 0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0xff, 0x63, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.101}");
        writeData({0xf0, 0x0, 0x6, 0x38, 0xff, 0xff, 0x9, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x0, 0xff, 0xff, 0xa, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.102}");
        writeData({0xf0, 0x0, 0x7, 0x38, 0x0, 0x0, 0x2, 0x0, 0x0, 0xff, 0xff, 0xff, 0xb, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x42, 0x5c, 0x61, 0xff, 0xc, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.103}");
        writeData({0xf0, 0x0, 0x8, 0x38, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x77, 0x77, 0x77, 0xff, 0xd, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x2, 0x0, 0x5e, 0x73, 0x79, 0xff, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.104}");
        writeData({0xf0, 0x0, 0x9, 0x38, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, col_bg[0], col_bg[1], col_bg[2], 0xff, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0xa5, 0xff, 0xff, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.105}");
        writeData({0xf0, 0x0, 0xa, 0x38, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0xff, 0xff, 0xff, 0xff, 0x11, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0xff, 0xff, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.106}");
        writeData({0xf0, 0x0, 0xb, 0x38, 0xff, 0x12, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x3d, 0xff, 0x0, 0xff, 0x13, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.107}");
        writeData({0xf0, 0x0, 0xc, 0x38, 0x0, 0x3, 0x0, 0xff, 0x63, 0xff, 0xff, 0x14, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0x0, 0xff, 0xff, 0x15, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.108}");
        writeData({0xf0, 0x0, 0xd, 0x38, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x0, 0xff, 0xff, 0xff, 0x16, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x42, 0x5c, 0x61, 0xff, 0x17, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.109}");
        writeData({0xf0, 0x0, 0xe, 0x38, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x77, 0x77, 0x77, 0xff, 0x18, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x3, 0x0, 0x5e, 0x73, 0x79, 0xff, 0x19, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.110}");
        writeData({0xf0, 0x0, 0xf, 0x38, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1a, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x4, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.111}");
        writeData({0xf0, 0x0, 0x10, 0x38, 0x1b, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x19, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0xe, 0x0, 0x0, 0x0, 0x4, 0x0, 0x2, 0x0, 0x0, 0x0, 0x1c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, identifierByte, 0xbb, 0x0, 0x0, 0x1a, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.112}");
        writeData({0xf0, 0x0, 0x11, 0x12, 0x2, identifierByte, 0xbb, 0x0, 0x0, 0x1c, 0x1, 0x0, 0x0, 0x76, 0x72, 0x19, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}, "FMC-Init{l.113}");

        setLedBrightness(FMCLed::BACKLIGHT, 128);
        setLedBrightness(FMCLed::SCREEN_BACKLIGHT, 128);
        setLedBrightness(FMCLed::OVERALL_LEDS_BRIGHTNESS, 255);
        setAllLedsEnabled(false);
        showBackground(FMCBackgroundVariant::WINWING_LOGO);
        
        setLedBrightness(FMCLed::MCDU_FAIL, 1);
        setLedBrightness(FMCLed::PFP_FAIL, 1);
        
        if (!profile) {
            setProfileForCurrentAircraft();
        }

        if (!_ioRunning.load()) {
            _ioRunning.store(true);
            _ioThread = std::thread([this]{ this->ioThreadMain(); });
        }
        
        return true;
    }
    
    return false;
}

void ProductFMC::disconnect() {
    if (_ioRunning.load()) {
        _ioRunning.store(false);
        _ioCv.notify_all();
        if (_ioThread.joinable()) _ioThread.join();
    }

    setLedBrightness(FMCLed::BACKLIGHT, 0);
    setLedBrightness(FMCLed::SCREEN_BACKLIGHT, 0);
    setAllLedsEnabled(false);

    unloadProfile();
    USBDevice::disconnect();
}

void ProductFMC::unloadProfile() {
    profileReady = false;
    
    if (!profile) {
        return;
    }
    
    delete profile;
    profile = nullptr;

    allowFontUpdateOnce();
}

void ProductFMC::update() {
    if (!connected) {
        return;
    }
    
    if (!profile) {
        setProfileForCurrentAircraft();
        return;
    }
    
    USBDevice::update();
    updatePage();
}

void ProductFMC::didReceiveData(int reportId, uint8_t *report, int reportLength) {    
    if (!connected || !profile || !report || reportLength <= 0) {
        return;
    }
    
    if (reportId != 1 || reportLength < 13) { // We only handle report #1 for now.
#if DEBUG
//        printf("[%s] Ignoring reportId %d, length %d\n", classIdentifier(), reportId, reportLength);
//        printf("[%s] Data (hex): ", classIdentifier());
//        for (int i = 0; i < reportLength; ++i) {
//            printf("%02X ", report[i]);
//        }
//        printf("\n");
#endif
        return;
    }
    
    uint64_t buttonsLo = 0;
    uint32_t buttonsHi = 0;
    for (int i = 0; i < 8; ++i) {
        buttonsLo |= ((uint64_t)report[i+1]) << (8 * i);
    }
    for (int i = 0; i < 4; ++i) {
        buttonsHi |= ((uint32_t)report[i+9]) << (8 * i);
    }
    
    if (buttonsLo == lastButtonStateLo && buttonsHi == lastButtonStateHi) {
        return;
    }
    
    lastButtonStateLo = buttonsLo;
    lastButtonStateHi = buttonsHi;
    
    for (int i = 0; i < 96; ++i) {
        bool pressed;
        
        if (i < 64) {
            pressed = (buttonsLo >> i) & 1;
        } else {
            pressed = (buttonsHi >> (i - 64)) & 1;
        }
        
        bool pressedButtonIndexExists = pressedButtonIndices.find(i) != pressedButtonIndices.end();
        XPLMCommandPhase command = -1;
        if (pressed && !pressedButtonIndexExists) {
            command = xplm_CommandBegin;
        }
        else if (pressed && pressedButtonIndexExists) {
            command = xplm_CommandContinue;
        }
        else if (!pressed && pressedButtonIndexExists) {
            command = xplm_CommandEnd;
        }
        
        if (command < 0) {
            continue;
        }
        
        if (command == xplm_CommandBegin) {
            pressedButtonIndices.insert(i);
        }
        
        FMCKey key = FMCHardwareMapping::ButtonIdentifierForIndex(hardwareType, i);
        const std::vector<FMCButtonDef>& currentButtonDefs = profile->buttonDefs();
        auto it = std::find_if(currentButtonDefs.begin(), currentButtonDefs.end(), [&](const FMCButtonDef& def) {
            return std::visit([&](auto&& k) {
                using T = std::decay_t<decltype(k)>;
                if constexpr (std::is_same_v<T, FMCKey>)
                    return k == key;
                else
                    return std::find(k.begin(), k.end(), key) != k.end();
            }, def.key);
        });
        if (it != currentButtonDefs.end()) {
            profile->buttonPressed(&*it, command);
        }
        
        if (command == xplm_CommandEnd) {
            pressedButtonIndices.erase(i);
        }
    }
}

void ProductFMC::updatePage() {
    const double now = XPLMGetElapsedTime();
    const double minPeriod = 1.0 / 25.0; // ~25 Hz

    if (now - _lastPush < minPeriod) return;

    auto drm = Dataref::getInstance();
    for (const std::string& dr : profile->displayDatarefs()) {
        if (!lastUpdateCycle || drm->getCachedLastUpdate(dr.c_str()) > lastUpdateCycle) {
            profile->updatePage(page);
            lastUpdateCycle = XPLMGetCycleNumber();
            draw();
            _lastPush = now;
            break;
        }
    }
}

void ProductFMC::ioThreadMain()
{
    auto drain = [&]() {
        std::unique_lock<std::mutex> lk(_ioMx);
        _ioCv.wait_for(lk, std::chrono::milliseconds(5), [&]{
            return !_ioQueue.empty() || !_ioRunning.load();
        });
        if (!_ioRunning.load()) return std::deque<IoCmd>{};
        std::deque<IoCmd> local; local.swap(_ioQueue);
        return local;
    };

    while (_ioRunning.load()) {
        auto q = drain();
        if (!_ioRunning.load()) break;

        // Coalesce simple: on applique dans l’ordre, mais on filtre les redondances
        for (auto& c : q) {
            switch (c.type) {
                case IoCmd::SetLedBrightness: {
                    // envoi immédiat (peu coûteux)
                    std::lock_guard<std::mutex> g(_usbWriteMx);
                    writeData({0x02, identifierByte, 0xbb, 0x00, 0x00, 0x03, 0x49,
                               c.a, c.b, 0x00, 0x00, 0x00, 0x00, 0x00},
                              "FMC-setLedBrightness{worker}");
                } break;

                case IoCmd::ClearScreen: {
                    // 16 x lignes blanches comme avant
                    std::vector<uint8_t> blank; blank.reserve(1 + 3*PageCharsPerLine);
                    blank.push_back(0xf2);
                    for (int i=0;i<PageCharsPerLine;++i){ blank.push_back(0x42); blank.push_back(0x00); blank.push_back(' '); }
                    std::lock_guard<std::mutex> g(_usbWriteMx);
                    for (int i=0;i<16;++i) writeData(blank, "FMC-Blank Line{worker}");
                } break;

                case IoCmd::ShowBackground: {
                    std::lock_guard<std::mutex> g(_usbWriteMx);
                    writeData(c.payload, "FMC-showBackground{worker}");
                } break;

                case IoCmd::UploadFont: {
                    // Rare → on pousse tout d'un coup
                    std::lock_guard<std::mutex> g(_usbWriteMx);
                    for (auto& chunk : c.payloadChunks) {
                        if (!chunk.empty()) writeData(chunk, "FMC-Font Bytes{worker}");
                    }
                } break;

                case IoCmd::UsbBuffer: {
                    // Rate-limit ~25 Hz + diff
                    const double now = XPLMGetElapsedTime();
                    if (c.payload == _sentFrame) break;           // pas de changement
                    if ((now - _lastUsbTs) < _minUsbPeriod) break;// trop tôt

                    // segmentation en blocs 63B + 0xF2 + padding
                    auto buf = c.payload;
                    std::lock_guard<std::mutex> g(_usbWriteMx);
                    while (!buf.empty()) {
                        size_t n = std::min<size_t>(63, buf.size());
                        std::vector<uint8_t> usbBuf(buf.begin(), buf.begin()+n);
                        usbBuf.insert(usbBuf.begin(), 0xf2);
                        if (n < 63) usbBuf.insert(usbBuf.end(), 63 - n, 0);
                        writeData(usbBuf, "FMC-Usb Buffer{worker}");
                        buf.erase(buf.begin(), buf.begin()+n);
                    }
                    _sentFrame = c.payload;
                    _lastUsbTs = now;
                } break;
            }
        }
    }
}

void ProductFMC::draw(const std::vector<std::vector<char>>* pagePtr) {
    const auto &p = pagePtr ? *pagePtr : page;
    std::vector<uint8_t> buf; buf.reserve(PageLines*PageCharsPerLine*3*2);

    for (int i = 0; i < ProductFMC::PageLines; ++i) {
        for (int j = 0; j < ProductFMC::PageCharsPerLine; ++j) {
            char color = p[i][j * ProductFMC::PageBytesPerChar];
            bool fontSmall = p[i][j * ProductFMC::PageBytesPerChar + 1];
            auto [lo, hi] = dataFromColFont(color, fontSmall);
            buf.push_back(lo); buf.push_back(hi);

            char val = p[i][j * ProductFMC::PageBytesPerChar + ProductFMC::PageBytesPerChar - 1];
            profile->mapCharacter(&buf, val, fontSmall);
        }
    }

    if (buf == _lastBuf) return;   // diff côté producteur (pour limiter la queue)
    _lastBuf = buf;

    // On remet le framing 63B dans le worker (coalescing + rate-limit global)
    qUsbBuffer(buf);
}

std::pair<uint8_t, uint8_t> ProductFMC::dataFromColFont(char color, bool fontSmall) {
    if (!profile) {
        return {0x42, 0x00}; // Default white
    }
    
    const std::map<char, FMCTextColor>& col_map = profile->colorMap();

    auto it = col_map.find(color);
    int value = it != col_map.end() ? it->second : FMCTextColor::COLOR_WHITE;
    if (fontSmall) {
        value += 0x016b;
    }

    return {static_cast<uint8_t>(value & 0xFF), static_cast<uint8_t>((value >> 8) & 0xFF)};
}

void ProductFMC::writeLineToPage(std::vector<std::vector<char>>& page, int line, int pos, const std::string &text, char color, bool fontSmall) {
    if (line < 0 || line >= ProductFMC::PageLines) {
        debug("Not writing line %i: Line number is out of range!\n", line);
        return;
    }
    if (pos < 0 || pos + text.length() > ProductFMC::PageCharsPerLine) {
        debug("Not writing line %i: Position number (%i) is out of range!\n", line, pos);
        return;
    }
    if (text.length() > ProductFMC::PageCharsPerLine) {
        debug("Not writing line %i: Text is too long (%lu) for line.\n", line, text.length());
        return;
    }

    pos = pos * ProductFMC::PageBytesPerChar;
    for (size_t c = 0; c < text.length(); ++c) {
        page[line][pos + c * ProductFMC::PageBytesPerChar] = color;
        page[line][pos + c * ProductFMC::PageBytesPerChar + 1] = fontSmall;
        page[line][pos + c * ProductFMC::PageBytesPerChar + ProductFMC::PageBytesPerChar - 1] = text[c];
    }
}

void ProductFMC::clearDisplay() {
    qClear();
}

namespace {
    // FNV-1a 32-bit sur un vecteur de vecteurs d’octets
    uint32_t fnv1a32(const std::vector<std::vector<unsigned char>>& ff) {
        uint32_t h = 2166136261u;
        for (const auto& chunk : ff) {
            for (unsigned char b : chunk) {
                h ^= b;
                h *= 16777619u;
            }
        }
        return h;
    }
}


void ProductFMC::setFont(std::vector<std::vector<unsigned char>> font) {
    if (!fontUpdatingEnabled) return;

    const uint32_t crc = fnv1a32(font);
    const double now = XPLMGetElapsedTime();
    if (_fontEverSent && crc == _fontCrc) return;
    if (_fontEverSent && (now - _fontLastTs) < 2.0) return;

    qUploadFont(font);      // <-- au lieu de writeData(...) dans la boucle

    _fontCrc = crc;
    _fontEverSent = true;
    _fontLastTs = now;
    fontUpdatingEnabled = false;
}

void ProductFMC::showBackground(FMCBackgroundVariant variant) {
    // Construit le paquet "image de fond" en fonction du variant
    std::vector<uint8_t> data;

    switch (variant) {
        case FMCBackgroundVariant::GRAY:
            data = {0xf0, 0x00, 0x02, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x53, 0x20, 0x07, 0x00};
            break;

        case FMCBackgroundVariant::BLACK:
            data = {0xf0, 0x00, 0x03, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0xfd, 0x24, 0x07, 0x00};
            break;

        case FMCBackgroundVariant::RED:
            data = {0xf0, 0x00, 0x04, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x55, 0x29, 0x07, 0x00};
            break;

        case FMCBackgroundVariant::GREEN:
            data = {0xf0, 0x00, 0x06, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0xad, 0x95, 0x09, 0x00};
            break;

        case FMCBackgroundVariant::BLUE:
            data = {0xf0, 0x00, 0x07, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0xa7, 0x9b, 0x09, 0x00};
            break;

        case FMCBackgroundVariant::YELLOW:
            data = {0xf0, 0x00, 0x08, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x09, 0xa1, 0x09, 0x00};
            break;

        case FMCBackgroundVariant::PURPLE:
            data = {0xf0, 0x00, 0x09, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x05, 0xa7, 0x09, 0x00};
            break;

        case FMCBackgroundVariant::WINWING_LOGO:
            data = {0xf0, 0x00, 0x0a, 0x12, identifierByte, 0xbb, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0xd4, 0xac, 0x09, 0x00};
            break;

        default:
            return; // variant inconnu
    }

    // Suffixe "extra" (inchangé) à concaténer
    std::vector<uint8_t> extra = {
        0x00, 0x01, 0x00, 0x00, 0x00, static_cast<uint8_t>(0x0c + static_cast<int>(variant)), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    data.insert(data.end(), extra.begin(), extra.end());

    // IMPORTANT : on passe par le worker I/O (pas d'accès direct USB ici)
    qBackground(variant, data);
}

void ProductFMC::setAllLedsEnabled(bool enable) {
    unsigned char start = FMCLed::_PFP_START;
    unsigned char end = FMCLed::_PFP_END;
    
    if (hardwareType == FMCHardwareType::HARDWARE_MCDU) {
        start = FMCLed::_MCDU_START;
        end = FMCLed::_MCDU_END;
    }
    
    for (unsigned char i = start; i <= end; ++i) {
        FMCLed led = static_cast<FMCLed>(i);
        setLedBrightness(led, enable ? 1 : 0);
    }
}

void ProductFMC::setLedBrightness(FMCLed led, uint8_t brightness) {
    if (led > FMCLed::OVERALL_LEDS_BRIGHTNESS && hardwareType == FMCHardwareType::HARDWARE_MCDU && led >= FMCLed::_PFP_START && led <= FMCLed::_PFP_END) {
        // Tried setting a PFP led on MCDU hardware, ignore.
        return;
    }
    else if (led > FMCLed::OVERALL_LEDS_BRIGHTNESS && hardwareType != FMCHardwareType::HARDWARE_MCDU && led >= FMCLed::_MCDU_START && led <= FMCLed::_MCDU_END) {
        // Tried setting a MCDU led on PFP hardware, ignore.
        return;
    }
    
    qLed(led, brightness);
}
