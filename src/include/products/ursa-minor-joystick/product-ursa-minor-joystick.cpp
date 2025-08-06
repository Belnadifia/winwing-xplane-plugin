#include "product-ursa-minor-joystick.h"
#include "dataref.h"
#include "appstate.h"
#include <cmath>
#include <algorithm>

ProductUrsaMinorJoystick::ProductUrsaMinorJoystick(HIDDeviceHandle hidDevice, uint16_t vendorId, uint16_t productId, std::string vendorName, std::string productName) : USBDevice(hidDevice, vendorId, productId, vendorName, productName) {
    connect();
}

ProductUrsaMinorJoystick::~ProductUrsaMinorJoystick() {
    disconnect();
}

const char* ProductUrsaMinorJoystick::classIdentifier() {
    return "Product-UrsaMinorJoystick";
}

bool ProductUrsaMinorJoystick::connect() {
    if (USBDevice::connect()) {
        setLedBrightness(0);
        setVibration(0);
        profileReady = true;
        return true;
    }

    return false;
}

void ProductUrsaMinorJoystick::disconnect() {
    setLedBrightness(0);
    setVibration(0);
    
    USBDevice::disconnect();
    
    Dataref::getInstance()->unbind("sim/cockpit/electrical/avionics_on");
    Dataref::getInstance()->unbind("AirbusFBW/PanelBrightnessLevel");
    Dataref::getInstance()->unbind("sim/flightmodel/failures/onground_any");
    didInitializeDatarefs = false;
}

void ProductUrsaMinorJoystick::update() {
    if (!connected) {
        return;
    }
    
    if (!didInitializeDatarefs) {
        initializeDatarefs();
    }
    
    USBDevice::update();
    
    if (Dataref::getInstance()->getCached<bool>("sim/flightmodel/failures/onground_any") && Dataref::getInstance()->getCached<bool>("sim/cockpit/electrical/avionics_on")) {
        float gForce = Dataref::getInstance()->get<float>("sim/flightmodel/forces/g_nrml");
        float delta = fabs(gForce - lastGForce);
        lastGForce = gForce;
        
        uint8_t vibration = (uint8_t)std::min(255.0f, delta * 400.0f);
        if (vibration < 8) {
            vibration = 0;
        }

        setVibration(vibration);
        lastVibration = vibration;
    }
}

bool ProductUrsaMinorJoystick::setVibration(uint8_t vibration) {
    return writeData({0x02, 7, 191, 0, 0, 3, 0x49, 0, vibration, 0, 0, 0, 0, 0});
}

bool ProductUrsaMinorJoystick::setLedBrightness(uint8_t brightness) {
    return writeData({0x02, 0x20, 0xbb, 0, 0, 3, 0x49, 0, brightness, 0, 0, 0, 0, 0});
}


void ProductUrsaMinorJoystick::initializeDatarefs() {
    if (!Dataref::getInstance()->exists("AirbusFBW/PanelBrightnessLevel")) {
        return;
    }
    
    didInitializeDatarefs = true;
    
    Dataref::getInstance()->monitorExistingDataref<float>("AirbusFBW/PanelBrightnessLevel", [this](float brightness) {
        bool hasPower = Dataref::getInstance()->get<bool>("sim/cockpit/electrical/avionics_on");
        uint8_t target = hasPower ? brightness * 255.0f : 0;
        setLedBrightness(target);
        
        if (!hasPower) {
            setVibration(0);
        }
    });
    
    Dataref::getInstance()->monitorExistingDataref<bool>("sim/cockpit/electrical/avionics_on", [this](bool poweredOn) {
        Dataref::getInstance()->executeChangedCallbacksForDataref("AirbusFBW/PanelBrightnessLevel");
    });
    
    Dataref::getInstance()->monitorExistingDataref<bool>("sim/flightmodel/failures/onground_any", [this](bool wheelsOnGround) {        
        if (!wheelsOnGround && lastVibration > 0) {
            lastVibration = 0;
            setVibration(lastVibration);
        }
    });
}
