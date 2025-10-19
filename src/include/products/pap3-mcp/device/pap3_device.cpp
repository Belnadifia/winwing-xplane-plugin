#include "pap3_device.h"
#include "pap3_demo.h"
#include "transport.h"
#include "../lcd/compose.h"
#include "../profiles/profile_factory.h"
#include "../aircraft/pap3_aircraft.h"
#include "../menu/pap3_menu.h"

#include "inputs.h"
#include "usbcontroller.h"

#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

// Define PAP3_HAS_DEMO when pap3_demo.cpp is linked; otherwise we provide a no-op.
#ifndef PAP3_HAS_DEMO
namespace pap3 { namespace device {
    class PAP3Device;                // fwd-decl (global, not in anonymous ns)
    inline void StartPap3Demo(PAP3Device*) {}  // no-op when demo is not linked
} } // namespace pap3::device
#endif

namespace {

struct EncDef { std::uint8_t posOff; const char* name; };
constexpr std::array<EncDef, 6> kEncDefs{{
    {0x15, "CRS CAPT"},
    {0x17, "SPD"},
    {0x19, "HDG"},
    {0x1B, "ALT"},
    {0x1D, "V/S"},
    {0x1F, "CRS FO"},
}};
inline const char* LookupEncoderName(std::uint8_t posOff) {
    for (const auto& e : kEncDefs) if (e.posOff == posOff) return e.name;
    return nullptr;
}

#ifndef PAP3_HAS_DEMO
namespace pap3 { namespace device {
    class PAP3Device;
    inline void StartPap3Demo(PAP3Device*) {}
} }
#endif

} // anon

namespace pap3::device {

using transport::DevicePtr;
using transport::sendATSolenoid;
using transport::sendDimming;
using transport::sendLcdCommit;
using transport::sendLcdEmptyFrame;
using transport::sendLcdInit;
using transport::sendLcdPayload;
using transport::setWriter;
using transport::writerUsbWriteData;

using aircraft::PAP3AircraftProfile;
using aircraft::ProfileFactory;
namespace lcdc = lcd::compose;

// -----------------------------------------------------------------------------
// Construction / destruction
// -----------------------------------------------------------------------------
PAP3Device::PAP3Device(HIDDeviceHandle hidDevice,
                       std::uint16_t  vendorId,
                       std::uint16_t  productId,
                       const std::string& vendorName,
                       const std::string& productName)
: USBDevice(hidDevice, vendorId, productId, vendorName, productName)
, _seq(5)
{
    ensureWriterInstalled();
    debug_force("PAP3Device constructed - vendorId: 0x%04X, productId: 0x%04X\n", vendorId, productId);

    if (USBDevice::connect()) {
        debug_force("PAP3Device connected\n");
        runStartupSequence();
    }
}

PAP3Device::~PAP3Device()
{
    _inputs.onEncoders    = nullptr;
    _inputs.onButtons     = nullptr;
    _inputs.onSwitchEdges = nullptr;
    _inputs.onLightSensor = nullptr;
    _inputs.onRaw         = nullptr;

    if (_ioRunning.exchange(false)) {
        _ioCv.notify_all();
        if (_ioThread.joinable()) _ioThread.join();
    }
    setDimming(0, 0);
    setDimming(1, 0);
    setDimming(2, 0);

    USBDevice::disconnect();
}

// -----------------------------------------------------------------------------
// Internal helpers
// -----------------------------------------------------------------------------
void PAP3Device::ensureWriterInstalled() const {
    setWriter(&writerUsbWriteData);
}

void PAP3Device::allLedsOff() {
    static const std::uint8_t kAllLedIds[] = {
        0x03, 0x04, 0x05, 0x06,
        0x07, 0x08, 0x09, 0x0A,
        0x0B, 0x0C, 0x0D, 0x0E,
        0x0F, 0x10, 0x11, 0x12,
        0x13
    };
    for (auto id : kAllLedIds) qSetLed(id, false);
}

bool PAP3Device::waitForInitialSwitchSnapshot(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lk(_snapshotMx);
    return _snapshotCv.wait_for(lk, timeout, [this]{ return _haveInitialReport; });
}

// -----------------------------------------------------------------------------
// Startup sequence (lisible, cohérent)
// -----------------------------------------------------------------------------
void PAP3Device::runStartupSequence()
{
    // 1) LEDs OFF
    allLedsOff();

    // 2) LCD init + clear
    sendLcdInit(static_cast<DevicePtr>(this), _seq);
    {
        std::vector<std::uint8_t> lcd32(32, 0x00);
        sendLcdPayload(static_cast<DevicePtr>(this), _seq, lcd32);
        sendLcdEmptyFrame(static_cast<DevicePtr>(this), _seq);
        sendLcdEmptyFrame(static_cast<DevicePtr>(this), _seq);
        sendLcdCommit(static_cast<DevicePtr>(this), _seq);
    }

    // 3) Dimming par défaut
    sendDimming(static_cast<DevicePtr>(this), 0, 255); // Backlight
    sendDimming(static_cast<DevicePtr>(this), 1, 255); // LCD
    sendDimming(static_cast<DevicePtr>(this), 2, 255); // LEDs

    // 4) Config illumination
    _illumCfg.backlight.mode         = IllumMode::Aircraft;
    _illumCfg.backlight.fixedPercent = 100;
    _illumCfg.lcd.mode               = IllumMode::Aircraft;
    _illumCfg.lcd.fixedPercent       = 100;
    _illumCfg.leds.mode              = IllumMode::Aircraft;
    _illumCfg.leds.fixedPercent      = 100;
    _illum.setConfig(_illumCfg);

    _illum.setAircraftPercents(_userBacklightPct, 100, 100);
    _illum.setLightSensorPercent(100);
    _illum.setPowerAvailable(true);

    // Appliquer la 1re frame de dimming
    {
        auto out = _illum.compute();
        if (out.changed[0]) qSetDimming(0, out.raw[0]);
        if (out.changed[1]) qSetDimming(1, out.raw[1]);
        if (out.changed[2]) qSetDimming(2, out.raw[2]);
    }

    // 5) Inputs : callbacks + worker I/O
    setupInputCallbacks();
    if (!_ioRunning.load()) {
        _ioRunning.store(true);
        _ioThread = std::thread([this]{ this->ioThreadMain(); });
    }

    // 6) Attendre une 1re photo des switches (snapshot HID brut)
    const bool gotSnapshot = waitForInitialSwitchSnapshot(std::chrono::milliseconds(1000));
    if (!gotSnapshot) {
        debug_force("[PAP3] WARNING: no initial switch snapshot within timeout -> continuing\n");
    }

    // 7) Détecter + démarrer le profil
    _profile = ProfileFactory::detect();
    if (_profile) {
        profileReady = true;
        _profile->attachDevice(this);

        // Bloquer l’application des frames sim pendant le boot
        _suppressApplyState = true;

        _profile->start([this](const aircraft::State& st) {
            if (_suppressApplyState) return;
            this->applyState(st);
        });

        // 8) Aligner le sim sur le hardware (snapshot si dispo)
        if (_haveInitialReport && !_initialReport.empty()) {
            const auto* data = _initialReport.data();
            const auto len = static_cast<int>(_initialReport.size());
            debug_force("[PAP3] Applying initial hardware snapshot (%d bytes)\n", len);
            _profile->syncSimToHardwareFromRaw(data, len);
            _pendingInitialHardwareSync = false;
            _didStartupSync = true;
        } else {
            debug_force("[PAP3] No raw snapshot yet -> will apply once received\n");
            _pendingInitialHardwareSync = true;
        }

        // Débloquer les frames sim
        _suppressApplyState = false;

        // Forcer une frame sim pour MAJ LEDs/LCD proprement
        _profile->tick();
    } else {
        StartPap3Demo(this);
        profileReady = true;
    }
    
    debug_force("[PAP3] Startup sequence complete - profile %s\n", _profile ? "ready" : "not found");

    updatePower(); // met à jour dimming + solénoïde (selon power mask)
}

// -----------------------------------------------------------------------------
// Public helpers (transport delegates)
// -----------------------------------------------------------------------------
bool PAP3Device::lcdInit() { return sendLcdInit(static_cast<DevicePtr>(this), _seq); }
bool PAP3Device::lcdSendPayload(const std::vector<std::uint8_t>& lcd32) {
    return sendLcdPayload(static_cast<DevicePtr>(this), _seq, lcd32);
}
bool PAP3Device::lcdSendEmpty() { return sendLcdEmptyFrame(static_cast<DevicePtr>(this), _seq); }
bool PAP3Device::lcdCommit() { return sendLcdCommit(static_cast<DevicePtr>(this), _seq); }
bool PAP3Device::setLed(std::uint8_t ledId, bool on) { return transport::sendLed(static_cast<DevicePtr>(this), ledId, on); }
bool PAP3Device::setDimming(std::uint8_t channel, std::uint8_t value) { return sendDimming(static_cast<DevicePtr>(this), channel, value); }
bool PAP3Device::setATSolenoid(bool on) { return sendATSolenoid(static_cast<DevicePtr>(this), on); }
void PAP3Device::setAtHardwareIntent(bool wantsOn) {
    _atHardwareIntent = wantsOn;
    _haveAtHardwareIntent = true;
    if (!wantsOn) {
        _pendingAtArmDrop = false;
        _pendingAtArmDropRefusal = false;
    }
}

void PAP3Device::didReceiveData(int /*reportId*/, std::uint8_t* report, int reportLength) {
    onHidInputReport(report, reportLength);
}

void PAP3Device::onHidInputReport(const std::uint8_t* report, int len)
{
    // 1) Capture one-shot du snapshot initial (brut)
    if (!_haveInitialReport && report && len > 0) {
        std::vector<std::uint8_t> freshSnapshot(report, report + len);
        {
            std::lock_guard<std::mutex> lk(_snapshotMx);
            _initialReport = std::move(freshSnapshot);
            _haveInitialReport = true;
        }
        _snapshotCv.notify_all();
        debug_force("[PAP3] Captured initial switch snapshot (%d bytes)\n", len);
        if (_pendingInitialHardwareSync && _profile) {
            debug_force("[PAP3] Applying captured hardware snapshot to sim (deferred)\n");
            _profile->syncSimToHardwareFromRaw(_initialReport.data(),
                                               static_cast<int>(_initialReport.size()));
            _pendingInitialHardwareSync = false;
            _didStartupSync = true;
        }
    }

    // 2) Décodage normal (edges)
    _inputs.decode(report, len);
}

// -----------------------------------------------------------------------------
// Inputs wiring
// -----------------------------------------------------------------------------
void PAP3Device::setupInputCallbacks()
{
    _inputs.setWatchedSwitchOffsets({0x01, 0x02, 0x03, 0x04, 0x05, 0x06});
    _inputs.setLightSensorOffset(0x14);

    _inputs.onEncoders = [this](const Inputs::EncodersFrame& e) {
        struct Item { std::int8_t d; std::uint8_t off; };
        const Item items[] = {
            {e.d_crsC, 0x15}, {e.d_spd , 0x17}, {e.d_hdg , 0x19},
            {e.d_alt , 0x1B}, {e.d_vvi , 0x1D}, {e.d_crsF, 0x1F},
        };

        for (const auto& it : items) {
            if (it.d == 0) continue;

            if (const char* name = LookupEncoderName(it.off)) {
                debug_force("[PAP3][ENC] %s: delta=%+d\n", name, static_cast<int>(it.d));
            } else {
                debug_force("[PAP3][ENC] off=0x%02X: delta=%+d\n", it.off, static_cast<int>(it.d));
            }

            if (_profile) _profile->onEncoderDelta(it.off, it.d);
        }
    };

    _inputs.onLightSensor = [](std::uint8_t /*lux*/) {};

    _inputs.onSwitchEdges = [this](std::uint8_t off, std::uint8_t changedSet, std::uint8_t changedClr) {
        auto forward = [this, off](uint8_t bit, bool pressed){
            if (_profile) _profile->onButton(off, bit, pressed);
        };
        for (std::uint8_t m = 1; m; m <<= 1) {
            if (changedSet & m)  forward(m, true);
            if (changedClr & m)  forward(m, false);
        }
    };
}

// -----------------------------------------------------------------------------
// Illumination & power
// -----------------------------------------------------------------------------
void PAP3Device::updatePower()
{
    const std::uint8_t mask = _profile ? _profile->mcpPowerMask() : 0x00;

    const bool powered = (mask & 0x01) != 0;
    _illum.setPowerAvailable(powered);

    const auto out = _illum.compute();
    if (out.changed[0]) qSetDimming(0, out.raw[0]); // Backlight
    if (out.changed[1]) qSetDimming(1, out.raw[1]); // LCD
    if (out.changed[2]) qSetDimming(2, out.raw[2]); // LEDs

    const bool solenoidOn = (mask & 0x06) != 0; // any DC bus
    if (!solenoidOn && _solenoidPowerAvailable) {
        if (_atPulsePending && _atPulseFL) {
            XPLMDestroyFlightLoop(_atPulseFL);
            _atPulseFL = nullptr;
        }
        _atPulsePending = false;
        _haveSimAtSample = false;
        _pendingAtArmDrop = false;
        _pendingAtArmDropRefusal = false;
    }
    _solenoidPowerAvailable = solenoidOn;
    qSetSolenoid(solenoidOn);
}

// -----------------------------------------------------------------------------
// Pulse A/T solenoid (OFF now, ON after delay via FlightLoop)
// -----------------------------------------------------------------------------
float PAP3Device::ATPulseFL(float /*elapsed*/, float /*since*/, int /*counter*/, void* refcon)
{
    auto* self = static_cast<PAP3Device*>(refcon);
    if (!self) return 0.0f;

    self->setATSolenoid(true);
    self->_atPulsePending = false;

    if (self->_atPulseFL) {
        XPLMDestroyFlightLoop(self->_atPulseFL);
        self->_atPulseFL = nullptr;
    }
    return 0.0f;
}

void PAP3Device::pulseATSolenoid(unsigned millis)
{
    if (_atPulsePending) return;
    if (!_solenoidPowerAvailable) {
        setATSolenoid(false);
        return;
    }

    setATSolenoid(false);

    if (!_atPulseFL) {
        XPLMCreateFlightLoop_t desc{};
        desc.structSize   = sizeof(desc);
        desc.phase        = xplm_FlightLoop_Phase_AfterFlightModel;
        desc.callbackFunc = &PAP3Device::ATPulseFL;
        desc.refcon       = this;
        _atPulseFL = XPLMCreateFlightLoop(&desc);
    }

    _atPulsePending = true;
    const float delaySec = static_cast<float>(millis) / 1000.0f;
    XPLMScheduleFlightLoop(_atPulseFL, delaySec, 1);
}

// -----------------------------------------------------------------------------
// Apply sim state -> device (LEDs/LCD/dimming) + watcher A/T
// -----------------------------------------------------------------------------
void PAP3Device::applyState(const aircraft::State& st)
{
    // --- LED mapping: PAP3 hardware IDs --------------------------------------
    static constexpr std::uint8_t kLedId_N1      = 0x03;
    static constexpr std::uint8_t kLedId_SPEED   = 0x04;
    static constexpr std::uint8_t kLedId_VNAV    = 0x05;
    static constexpr std::uint8_t kLedId_LVL_CHG = 0x06;
    static constexpr std::uint8_t kLedId_HDG_SEL = 0x07;
    static constexpr std::uint8_t kLedId_LNAV    = 0x08;
    static constexpr std::uint8_t kLedId_VORLOC  = 0x09;
    static constexpr std::uint8_t kLedId_APP     = 0x0A;
    static constexpr std::uint8_t kLedId_ALT_HLD = 0x0B;
    static constexpr std::uint8_t kLedId_VS      = 0x0C;
    static constexpr std::uint8_t kLedId_CMD_A   = 0x0D;
    static constexpr std::uint8_t kLedId_CWS_A   = 0x0E;
    static constexpr std::uint8_t kLedId_CMD_B   = 0x0F;
    static constexpr std::uint8_t kLedId_CWS_B   = 0x10;
    static constexpr std::uint8_t kLedId_AT_ARM  = 0x11;
    static constexpr std::uint8_t kLedId_MA_CAPT = 0x12;
    static constexpr std::uint8_t kLedId_MA_FO   = 0x13;

    // Push LED states from sim -> device
    qSetLed(kLedId_N1,      st.led.N1);
    qSetLed(kLedId_SPEED,   st.led.SPEED);
    qSetLed(kLedId_VNAV,    st.led.VNAV);
    qSetLed(kLedId_LVL_CHG, st.led.LVL_CHG);
    qSetLed(kLedId_HDG_SEL, st.led.HDG_SEL);
    qSetLed(kLedId_LNAV,    st.led.LNAV);
    qSetLed(kLedId_VORLOC,  st.led.VORLOC);
    qSetLed(kLedId_APP,     st.led.APP);
    qSetLed(kLedId_ALT_HLD, st.led.ALT_HLD);
    qSetLed(kLedId_VS,      st.led.V_S);
    qSetLed(kLedId_CMD_A,   st.led.CMD_A);
    qSetLed(kLedId_CWS_A,   st.led.CWS_A);
    qSetLed(kLedId_CMD_B,   st.led.CMD_B);
    qSetLed(kLedId_CWS_B,   st.led.CWS_B);
    qSetLed(kLedId_AT_ARM,  st.led.AT_ARM);
    qSetLed(kLedId_MA_CAPT, st.led.MA_CAPT);
    qSetLed(kLedId_MA_FO,   st.led.MA_FO);

    // --- LCD: State → Snapshot → 32B payload --------------------------------
    lcdc::Snapshot s{};
    s.spd     = st.spd;
    s.hdg     = st.hdg;
    s.alt     = st.alt;
    s.vvi     = st.vvi;
    s.showVvi = st.vviVisible;
    s.crsCapt = st.crsCapt;
    s.crsFo   = st.crsFo;
    s.digitA  = st.digitA;
    s.digitB  = st.digitB;

    const bool show = pap3menu::GetShowLcdLabels();
    s.lblIAS = show; s.lblHDG = show; s.lblVS  = show;

    s.dotSpd = s.dotAlt = s.dotVvi = s.dotCrsCapt = s.dotCrsFo = s.dotHdg = false;

    const auto payload = lcdc::build(s);
    std::vector<std::uint8_t> lcd32(payload.begin(), payload.end());
    qLcdPayload(lcd32);

    // Convert [0..1] -> [0..100]
    const auto clampPct = [](float v)->int{
        return std::clamp(static_cast<int>(std::lround(v * 100.0f)), 0, 100);
    };
    _illum.setAircraftPercents(clampPct(st.cockpitLights), clampPct(st.mcpBrightness), clampPct(st.ledsBrightness));

    updatePower();

    // --- watcher A/T (edge stable) ---
    bool simAtArmed = st.atArmOn;
    if (!simAtArmed && !_haveSimAtSample && st.led.AT_ARM) {
        // Before we record a baseline, fall back to annunciator to avoid false drop
        simAtArmed = true;
    }

    if (!_solenoidPowerAvailable) {
        _haveSimAtSample = false;
        _pendingAtArmDrop = false;
        _pendingAtArmDropRefusal = false;
        _lastSimAtArm = simAtArmed;
        _atStabilityCount = 0;
        return;
    }

    if (!_haveSimAtSample) {
        _haveSimAtSample = true;
        _lastSimAtArm = simAtArmed;
        _atStabilityCount = 0;
        return;
    }

    if (simAtArmed == _lastSimAtArm) {
        if (_atStabilityCount < 1000) _atStabilityCount++;
    } else {
        _atStabilityCount = 0;
        _pendingAtArmDrop = (_lastSimAtArm && !simAtArmed);
    }

    if (_pendingAtArmDrop && _didStartupSync && _atStabilityCount >= 2) {
        pulseATSolenoid(250);
        _pendingAtArmDrop = false;
        _pendingAtArmDropRefusal = false;
    }

    if (!_pendingAtArmDrop) {
        _pendingAtArmDropRefusal = false;
    }

    _lastSimAtArm = simAtArmed;
}

// -----------------------------------------------------------------------------
// Periodic update
// -----------------------------------------------------------------------------
void PAP3Device::update() {
    this->USBDevice::update();
    if (_profile) _profile->tick();
}

// -----------------------------------------------------------------------------
// Worker loop
// -----------------------------------------------------------------------------
void PAP3Device::ioThreadMain() {
    uint32_t pendLedBitmap = _sentLedBitmap;
    uint8_t  pendDim[3] = {_sentDimming[0], _sentDimming[1], _sentDimming[2]};
    bool     pendSol = _sentSolenoid;
    std::vector<uint8_t> pendLcd32 = _sentLcd32;

    auto drainQueue = [&](){
        std::unique_lock<std::mutex> lk(_ioMx);
        _ioCv.wait_for(lk, std::chrono::milliseconds(5), [&]{ return !_ioQueue.empty() || !_ioRunning.load(); });
        if (!_ioRunning.load()) return;
        while(!_ioQueue.empty()) {
            IoCmd c = std::move(_ioQueue.front());
            _ioQueue.pop_front();
            lk.unlock();

            switch (c.type) {
                case IoCmd::SetLed: {
                    const uint8_t idx = (c.a >= 0x03) ? (c.a - 0x03) : 0;
                    const uint32_t bit = (1u << idx);
                    if (c.b) pendLedBitmap |= bit; else pendLedBitmap &= ~bit;
                } break;
                case IoCmd::SetDimming: {
                    if (c.a < 3) pendDim[c.a] = c.b;
                } break;
                case IoCmd::SetATSolenoid: {
                    pendSol = (c.b != 0);
                } break;
                case IoCmd::LcdPayload: {
                    pendLcd32 = std::move(c.payload);
                } break;
            }

            lk.lock();
        }
    };

    using clock = std::chrono::steady_clock;
    auto lastLcdTx = clock::now();

    while(_ioRunning.load()) {
        drainQueue();
        if (!_ioRunning.load()) break;

        // 1) LEDs diffs
        uint32_t diff = pendLedBitmap ^ _sentLedBitmap;
        if (diff) {
            for (uint8_t i = 0; i < 32; ++i) {
                uint32_t bit = (1u << i);
                if (diff & bit) {
                    uint8_t ledId = 0x03 + i;
                    bool on = (pendLedBitmap & bit) != 0;
                    transport::sendLed(static_cast<transport::DevicePtr>(this), ledId, on);
                }
            }
            _sentLedBitmap = pendLedBitmap;
        }

        // 2) Dimming
        for (uint8_t ch = 0; ch < 3; ++ch) {
            if (_sentDimming[ch] != pendDim[ch]) {
                transport::sendDimming(static_cast<transport::DevicePtr>(this), ch, pendDim[ch]);
                _sentDimming[ch] = pendDim[ch];
            }
        }

        // 3) Solenoid
        if (_sentSolenoid != pendSol) {
            transport::sendATSolenoid(static_cast<transport::DevicePtr>(this), pendSol);
            _sentSolenoid = pendSol;
        }

        // 4) LCD coalescing + rate-limit
        const auto minPeriod = std::chrono::duration<double>(_minLcdPeriod);
        const auto now = clock::now();
        const bool timeOk = ((now - lastLcdTx) >= minPeriod);
        const bool havePend = !pendLcd32.empty();
        const bool changed = havePend && (pendLcd32 != _sentLcd32);

        if (changed && timeOk) {
            auto* dev = static_cast<transport::DevicePtr>(this);
            transport::sendLcdPayload(dev, _seq, pendLcd32);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            transport::sendLcdEmptyFrame(dev, _seq);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            transport::sendLcdEmptyFrame(dev, _seq);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            transport::sendLcdCommit(dev, _seq);

            _sentLcd32 = pendLcd32;
            lastLcdTx = now;
        }
    }
}

} // namespace pap3::device