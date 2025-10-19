#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_map>

#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include "inputs.h"
#include "illumination.h"
#include "transport.h" // for transport::DevicePtr
#include "usbdevice.h" // base

namespace pap3 {
namespace aircraft {
    struct State;
    class PAP3AircraftProfile;
}
}

namespace pap3::device {

class PAP3Device : public USBDevice {
public:
    PAP3Device(HIDDeviceHandle hidDevice,
               uint16_t vendorId,
               uint16_t productId,
               const std::string& vendorName,
               const std::string& productName);
    ~PAP3Device() override;

    // LCD ops
    bool lcdInit();
    bool lcdSendPayload(const std::vector<std::uint8_t>& lcd32);
    bool lcdSendEmpty();
    bool lcdCommit();

    // LEDs / dimming / solenoid
    bool setLed(std::uint8_t ledId, bool on);
    bool setDimming(std::uint8_t channel, std::uint8_t value);
    bool setATSolenoid(bool on);
    void setAtHardwareIntent(bool wantsOn);

    // Expose current sequence
    std::uint8_t currentSeq() const noexcept { return _seq; }

    void update() override;

    // Optional entry if a lower layer receives HID reports
    void onHidInputReport(const uint8_t* report, int len);
    void didReceiveData(int reportId, uint8_t* report, int reportLength) override;

    // One-shot solenoid pulse: OFF now, ON after delay
    void pulseATSolenoid(unsigned millis = 50);

private:
    // Writer bridge
    void ensureWriterInstalled() const;

    // Boot
    void runStartupSequence();
    void allLedsOff();

    // Illumination & power
    void updatePower();

    // Inputs wiring
    void setupInputCallbacks();
    bool waitForInitialSwitchSnapshot(std::chrono::milliseconds timeout);

    // Worker queue
    struct IoCmd {
        enum Type : uint8_t {
            SetLed,
            SetDimming,
            LcdPayload,
            SetATSolenoid
        } type;
        uint8_t a = 0;
        uint8_t b = 0;
        std::vector<uint8_t> payload;
    };
    void ioThreadMain();

    inline void qEnqueue(IoCmd&& c) {
        { std::lock_guard<std::mutex> lk(_ioMx); _ioQueue.emplace_back(std::move(c)); }
        _ioCv.notify_one();
    }
    inline void qSetLed(uint8_t ledId, bool on) {
        IoCmd c; c.type = IoCmd::SetLed; c.a = ledId; c.b = on ? 1 : 0; qEnqueue(std::move(c));
    }
    inline void qSetDimming(uint8_t channel, uint8_t value) {
        IoCmd c; c.type = IoCmd::SetDimming; c.a = channel; c.b = value; qEnqueue(std::move(c));
    }
    inline void qSetSolenoid(bool on) {
        IoCmd c; c.type = IoCmd::SetATSolenoid; c.b = on ? 1 : 0; qEnqueue(std::move(c));
    }
    inline void qLcdPayload(const std::vector<uint8_t>& lcd32) {
        IoCmd c; c.type = IoCmd::LcdPayload; c.payload = lcd32; qEnqueue(std::move(c));
    }

private:
    // Rolling non-zero sequence
    std::uint8_t _seq { 5 };

    // Illumination engine
    pap3::device::IlluminationEngine _illum{};
    pap3::device::IlluminationConfig _illumCfg{};
    int _userBacklightPct{100};

    // Profile bridge
    std::unique_ptr<pap3::aircraft::PAP3AircraftProfile> _profile;

    // Input decoding
    pap3::device::Inputs _inputs;

    // Snapshot boot
    std::mutex _snapshotMx;
    std::condition_variable _snapshotCv;
    std::vector<std::uint8_t> _initialReport;
    bool _haveInitialReport{false};
    bool _didStartupSync{false};
    bool _pendingInitialHardwareSync{false};

    // I/O worker
    std::thread              _ioThread;
    std::atomic<bool>        _ioRunning{false};
    std::mutex               _ioMx;
    std::condition_variable  _ioCv;
    std::deque<IoCmd>        _ioQueue;

    // Coalescing state actually sent to device
    uint32_t                 _sentLedBitmap = 0;
    uint8_t                  _sentDimming[3] = {255,255,255};
    bool                     _sentSolenoid = false;
    std::vector<uint8_t>     _sentLcd32;

    // LCD rate-limit
    float                    _minLcdPeriod = 1.f / 25.f; // ~25 Hz

    // FlightLoop for AT pulse
    XPLMFlightLoopID _atPulseFL{nullptr};
    bool _atPulsePending{false};
    static float ATPulseFL(float elapsed, float since, int counter, void* refcon);

    // Apply-state suppression during boot
    bool _suppressApplyState{false};

    // Stability watcher A/T (edge)
    bool _lastSimAtArm{true};
    int  _atStabilityCount{0};
    bool _pendingAtArmDrop{false};
    bool _pendingAtArmDropRefusal{false};
    bool _solenoidPowerAvailable{false};
    bool _haveSimAtSample{false};
    bool _atHardwareIntent{false};
    bool _haveAtHardwareIntent{false};

    void applyState(const pap3::aircraft::State& st);
};

} // namespace pap3::device