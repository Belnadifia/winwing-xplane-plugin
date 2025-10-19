#pragma once
#include "../pap3_aircraft.h"
#include <XPLMDataAccess.h>
#include <XPLMUtilities.h>
#include <vector>
#include <string>
#include <cstdint>
#include <cmath>

namespace pap3::aircraft {

class ZiboPAP3Profile final : public PAP3AircraftProfile {
public:
    using State = pap3::aircraft::State;

    ZiboPAP3Profile();
    ~ZiboPAP3Profile() override;

    // ---- PAP3AircraftProfile API ----
    bool  isEligible() const override;
    void  start(StateCallback onChanged) override;
    void  stop() override;
    State current() const override;
    void  tick() override;

    // Démarrages (compat, mais interne simple)
    void  syncSimToHardware() override;
    void  syncSimToHardwareFromRaw(const std::uint8_t* report, int len) override;

    uint8_t mcpPowerMask() const override;
    bool    mcpHasPower() const override;

    // Entrées device (on garde ces hooks pour ne pas toucher le device)
    void onButton(std::uint8_t off, std::uint8_t mask, bool pressed) override;
    void onEncoderDelta(std::uint8_t posOff, std::int8_t delta) override;

    // --- Bank angle (Zibo) ---
    int  readBankIndex() const;        // 0..4
    void nudgeBankAngleTo(int target); // 0..4

private:
    // --- Lecture sim -> State
    void  poll();

    // --- Cmd helpers
    void  execOnce(XPLMCommandRef cmd);
    void  repeatCmd(XPLMCommandRef inc, XPLMCommandRef dec, int8_t delta, int stepPerTick = 1);

    // --- Debounce/guards
    bool  debounce(float& lastTs, float minDeltaSec = 0.05f) const;

    // ---------- LOGIQUE CENTRALE ----------
    // Intent = unique source de vérité (ce que l’on veut dans le sim)
    struct Intent {
        // F/D: ON -> dataref pos = 1 ; OFF -> pos = 0
        bool fd_capt_on{false};
        bool fd_fo_on{false};
        // A/T: ARMED -> pos = 1 ; DISARMED -> pos = 0
        bool at_arm_on{false};
        // A/P DISENGAGE : ENGAGED -> pos = 0 ; DISENGAGED -> pos = 1
        bool ap_engaged{false};
    };

    void  applyIntent();                           // applique toutes les cibles
    void  seedFromSim();                           // initialise l’intent depuis le sim
    void  seedFromRaw(const std::uint8_t* r, int len); // initialise depuis snapshot HID

    // Toggler bas-niveau (aller à pos 0/1) + compat ancienne API
    static void toggleToPosIfNeeded(XPLMDataRef posRef, int desiredPos, XPLMCommandRef toggleCmd, float& guardTs, int& lastCommandedPos);
    static inline int  dr_get_pos(XPLMDataRef r) {
        if (!r) return 0;
        const XPLMDataTypeID ty = XPLMGetDataRefTypes(r);
        if (ty & xplmType_Int)    return XPLMGetDatai(r);
        if (ty & xplmType_Float)  return static_cast<int>(std::lround(XPLMGetDataf(r)));
        if (ty & xplmType_Double) return static_cast<int>(std::lround(XPLMGetDatad(r)));
        return XPLMGetDatai(r);
    }
    static inline bool dr_is_on(XPLMDataRef r) {
        if (!r) return false;
        const XPLMDataTypeID ty = XPLMGetDataRefTypes(r);
        if (ty & xplmType_Int)    return XPLMGetDatai(r) != 0;
        if (ty & xplmType_Float)  return XPLMGetDataf(r) > 0.5f;
        if (ty & xplmType_Double) return XPLMGetDatad(r) > 0.5;
        return XPLMGetDatai(r) != 0;
    }
    static inline bool bit(const uint8_t* r, int len, uint8_t off, uint8_t mask) {
        return (r && len > 0 && off < (uint8_t)len) && ((r[off] & mask) != 0);
    }

private:
    static constexpr int FD_ON_POS  = 1;
    static constexpr int FD_OFF_POS = 0;
    static constexpr int AT_ON_POS  = 1;
    static constexpr int AT_OFF_POS = 0;
    static constexpr int AP_ON_POS  = 0;
    static constexpr int AP_OFF_POS = 1;

    // ---- État temps-réel poussé au device
    State         _state{};
    StateCallback _cb;
    bool          _running{false};

    // ---- Intent matériel (remplace les anciennes variables éclatées)
    Intent        _intent{};
    bool          _haveIntent{false};
    bool          _pendingIntentApply{false};

    // ---- Datarefs presence check ----
    XPLMDataRef _drZiboCheck{nullptr}; // laminar/B738/zibomod/filename_list

    // --- MCP numeric values ---
    XPLMDataRef _drSpd{nullptr};
    XPLMDataRef _drHdg{nullptr};
    XPLMDataRef _drAlt{nullptr};
    XPLMDataRef _drVvi{nullptr};
    XPLMDataRef _drVviShow{nullptr};
    XPLMDataRef _drCrsCapt{nullptr};
    XPLMDataRef _drCrsFo{nullptr};

    // Illumination / brightness
    XPLMDataRef _drMcpBrightnessArr{nullptr}; // sim/cockpit2/electrical/instrument_brightness_ratio_manual
    XPLMDataRef _drCockpitLightsArr{nullptr}; // laminar/B738/electric/panel_brightness

    // MCP LCD special digits
    XPLMDataRef _drDigitA{nullptr}; // laminar/B738/mcp/digit_A
    XPLMDataRef _drDigitB{nullptr}; // laminar/B738/mcp/digit_8

    // --- LEDs annunciators ---
    XPLMDataRef _drLedN1{nullptr};
    XPLMDataRef _drLedSpd{nullptr};
    XPLMDataRef _drLedVnav{nullptr};
    XPLMDataRef _drLedLvlChg{nullptr};
    XPLMDataRef _drLedHdgSel{nullptr};
    XPLMDataRef _drLedLnav{nullptr};
    XPLMDataRef _drLedVorLoc{nullptr};
    XPLMDataRef _drLedApp{nullptr};
    XPLMDataRef _drLedAltHld{nullptr};
    XPLMDataRef _drLedVs{nullptr};
    XPLMDataRef _drLedCmdA{nullptr};
    XPLMDataRef _drLedCwsA{nullptr};
    XPLMDataRef _drLedCmdB{nullptr};
    XPLMDataRef _drLedCwsB{nullptr};
    XPLMDataRef _drLedAtArm{nullptr};
    XPLMDataRef _drLedMaCapt{nullptr};
    XPLMDataRef _drLedMaFo{nullptr};

    // --- Maintained switch positions (0/1) ---
    XPLMDataRef _drFDCaptPos{nullptr}; // laminar/B738/autopilot/flight_director_pos
    XPLMDataRef _drFDFoPos{nullptr};   // laminar/B738/autopilot/flight_director_fo_pos
    XPLMDataRef _drATArmPos{nullptr};  // laminar/B738/autopilot/autothrottle_arm_pos
    XPLMDataRef _drApDiscPos{nullptr}; // laminar/B738/autopilot/disconnect_pos

    // --- Toggle commands ---
    XPLMCommandRef _cmdFDCaptToggle{nullptr};
    XPLMCommandRef _cmdFDFoToggle{nullptr};
    XPLMCommandRef _cmdATArmToggle{nullptr};
    XPLMCommandRef _cmdApDiscToggle{nullptr};

    // --- Power bits / refs ---
    XPLMDataRef _drHasApPower{nullptr}; // sim/cockpit2/autopilot/autopilot_has_power
    XPLMDataRef _drDcBus1{nullptr};     // laminar/B738/electric/dc_bus1_status
    XPLMDataRef _drDcBus2{nullptr};     // laminar/B738/electric/dc_bus2_status

    // --- Bank angle (Zibo) ---
    XPLMDataRef    _drBankIdx{nullptr};          // laminar/B738/autopilot/bank_angle_pos|_sel
    XPLMCommandRef _cmdBankUp{nullptr};          // laminar/B738/autopilot/bank_angle_up
    XPLMCommandRef _cmdBankDn{nullptr};          // laminar/B738/autopilot/bank_angle_dn

    // --- Debounce & guards ---
    float _lastAtToggleTime{0.0f};
    float _lastApDiscToggleTime{0.0f};
    float _guardFdCaptTs{0.0f};
    float _guardFdFoTs{0.0f};
    float _guardAtTs{0.0f};
    float _guardApDiscTs{0.0f};
    int   _lastFdCaptCmdPos{-1};
    int   _lastFdFoCmdPos{-1};
    int   _lastAtCmdPos{-1};
    int   _lastApDiscCmdPos{-1};

    // Momentary buttons mapping (offset/mask -> cmd)
    struct BtnBinding {
        uint8_t       off;
        uint8_t       mask;
        XPLMCommandRef press{nullptr};
        XPLMCommandRef release{nullptr}; // optional
    };
    std::vector<BtnBinding> _btns;

    // Encoders mapping
    struct EncBinding {
        uint8_t       posOff;
        XPLMCommandRef inc{nullptr};
        XPLMCommandRef dec{nullptr};
        int           stepPerTick{1};
    };
    std::vector<EncBinding> _encs;
};

} // namespace pap3::aircraft