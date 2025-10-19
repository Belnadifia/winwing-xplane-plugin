#include "zibo_profile.h"
#include "../../device/pap3_device.h"

#include <algorithm>
#include <cmath>

#include <XPLMProcessing.h>
#include <XPLMDataAccess.h>
#include <XPLMUtilities.h>

namespace pap3::aircraft {

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static inline XPLMCommandRef Cmd(const char* s) { return XPLMFindCommand(s); }
static inline bool bit(const std::uint8_t* r, int len, std::uint8_t off, std::uint8_t mask) {
    return (r && len > 0 && off < (std::uint8_t)len) && ((r[off] & mask) != 0);
}

// -----------------------------------------------------------------------------
// Ctor / Dtor
// -----------------------------------------------------------------------------
ZiboPAP3Profile::ZiboPAP3Profile() {
    _drZiboCheck = XPLMFindDataRef("laminar/B738/zibomod/filename_list");

    // MCP values
    _drSpd     = XPLMFindDataRef("laminar/B738/autopilot/mcp_speed_dial_kts_mach");
    _drHdg     = XPLMFindDataRef("laminar/B738/autopilot/mcp_hdg_dial");
    _drAlt     = XPLMFindDataRef("laminar/B738/autopilot/mcp_alt_dial");
    _drVvi     = XPLMFindDataRef("sim/cockpit2/autopilot/vvi_dial_fpm");
    _drVviShow = XPLMFindDataRef("laminar/B738/autopilot/vvi_dial_show");
    _drCrsCapt = XPLMFindDataRef("laminar/B738/autopilot/course_pilot");
    _drCrsFo   = XPLMFindDataRef("laminar/B738/autopilot/course_copilot");

    // Brightness
    _drMcpBrightnessArr = XPLMFindDataRef("sim/cockpit2/electrical/instrument_brightness_ratio_manual");
    _drCockpitLightsArr = XPLMFindDataRef("laminar/B738/electric/panel_brightness");

    // LCD digits
    _drDigitA = XPLMFindDataRef("laminar/B738/mcp/digit_A");
    _drDigitB = XPLMFindDataRef("laminar/B738/mcp/digit_8");

    // LEDs
    _drLedN1     = XPLMFindDataRef("laminar/B738/autopilot/n1_status1");
    _drLedSpd    = XPLMFindDataRef("laminar/B738/autopilot/speed_status1");
    _drLedVnav   = XPLMFindDataRef("laminar/B738/autopilot/vnav_status1");
    _drLedLvlChg = XPLMFindDataRef("laminar/B738/autopilot/lvl_chg_status");
    _drLedHdgSel = XPLMFindDataRef("laminar/B738/autopilot/hdg_sel_status");
    _drLedLnav   = XPLMFindDataRef("laminar/B738/autopilot/lnav_status");
    _drLedVorLoc = XPLMFindDataRef("laminar/B738/autopilot/vorloc_status");
    _drLedApp    = XPLMFindDataRef("laminar/B738/autopilot/app_status");
    _drLedAltHld = XPLMFindDataRef("laminar/B738/autopilot/alt_hld_status");
    _drLedVs     = XPLMFindDataRef("laminar/B738/autopilot/vs_status");
    _drLedCmdA   = XPLMFindDataRef("laminar/B738/autopilot/cmd_a_status");
    _drLedCwsA   = XPLMFindDataRef("laminar/B738/autopilot/cws_a_status");
    _drLedCmdB   = XPLMFindDataRef("laminar/B738/autopilot/cmd_b_status");
    _drLedCwsB   = XPLMFindDataRef("laminar/B738/autopilot/cws_b_status");
    _drLedAtArm  = XPLMFindDataRef("laminar/B738/autopilot/autothrottle_status1");
    _drLedMaCapt = XPLMFindDataRef("laminar/B738/autopilot/master_capt_status");
    _drLedMaFo   = XPLMFindDataRef("laminar/B738/autopilot/master_fo_status");

    // Maintained positions (0/1)
    _drFDCaptPos = XPLMFindDataRef("laminar/B738/autopilot/flight_director_pos");
    _drFDFoPos   = XPLMFindDataRef("laminar/B738/autopilot/flight_director_fo_pos");
    _drATArmPos  = XPLMFindDataRef("laminar/B738/autopilot/autothrottle_arm_pos");
    _drApDiscPos = XPLMFindDataRef("laminar/B738/autopilot/disconnect_pos");

    // Toggles
    _cmdFDCaptToggle = Cmd("laminar/B738/autopilot/flight_director_toggle");
    _cmdFDFoToggle   = Cmd("laminar/B738/autopilot/flight_director_fo_toggle");
    _cmdATArmToggle  = Cmd("laminar/B738/autopilot/autothrottle_arm_toggle");
    _cmdApDiscToggle = Cmd("laminar/B738/autopilot/disconnect_toggle");

    // Power
    _drHasApPower = XPLMFindDataRef("sim/cockpit2/autopilot/autopilot_has_power");
    _drDcBus1     = XPLMFindDataRef("laminar/B738/electric/dc_bus1_status");
    _drDcBus2     = XPLMFindDataRef("laminar/B738/electric/dc_bus2_status");

    // Bank angle
    _drBankIdx = XPLMFindDataRef("laminar/B738/autopilot/bank_angle_pos");
    if (!_drBankIdx) _drBankIdx = XPLMFindDataRef("laminar/B738/autopilot/bank_angle_sel");
    _cmdBankUp = Cmd("laminar/B738/autopilot/bank_angle_up");
    _cmdBankDn = Cmd("laminar/B738/autopilot/bank_angle_dn");

    // Buttons (momentary)
    auto B = [&](uint8_t off, uint8_t mask, const char* press, const char* release=nullptr){
        _btns.push_back({off, mask, Cmd(press), release ? Cmd(release) : nullptr});
    };

    // 0x01
    B(0x01, 0x01, "laminar/B738/autopilot/n1_press");
    B(0x01, 0x02, "laminar/B738/autopilot/speed_press");
    B(0x01, 0x04, "laminar/B738/autopilot/vnav_press");
    B(0x01, 0x08, "laminar/B738/autopilot/lvl_chg_press");
    B(0x01, 0x10, "laminar/B738/autopilot/hdg_sel_press");
    B(0x01, 0x20, "laminar/B738/autopilot/lnav_press");
    B(0x01, 0x40, "laminar/B738/autopilot/vorloc_press");
    B(0x01, 0x80, "laminar/B738/autopilot/app_press");

    // 0x02
    B(0x02, 0x01, "laminar/B738/autopilot/alt_hld_press");
    B(0x02, 0x02, "laminar/B738/autopilot/vs_press");
    B(0x02, 0x04, "laminar/B738/autopilot/cmd_a_press");
    B(0x02, 0x08, "laminar/B738/autopilot/cws_a_press");
    B(0x02, 0x10, "laminar/B738/autopilot/cmd_b_press");
    B(0x02, 0x20, "laminar/B738/autopilot/cws_b_press");
    B(0x02, 0x40, "laminar/B738/autopilot/change_over_press");
    B(0x02, 0x80, "laminar/B738/autopilot/spd_interv");

    // 0x03
    B(0x03, 0x01, "laminar/B738/autopilot/alt_interv");

    // Encoders
    auto E = [&](uint8_t posOff, const char* inc, const char* dec, int step=1){
        _encs.push_back({posOff, Cmd(inc), Cmd(dec), step});
    };
    E(0x15, "laminar/B738/autopilot/course_pilot_up",     "laminar/B738/autopilot/course_pilot_dn");
    E(0x17, "sim/autopilot/airspeed_up",                  "sim/autopilot/airspeed_down");
    E(0x19, "laminar/B738/autopilot/heading_up",          "laminar/B738/autopilot/heading_dn");
    E(0x1B, "laminar/B738/autopilot/altitude_up",         "laminar/B738/autopilot/altitude_dn");
    E(0x1D, "sim/autopilot/vertical_speed_up",            "sim/autopilot/vertical_speed_down");
    E(0x1F, "laminar/B738/autopilot/course_copilot_up",   "laminar/B738/autopilot/course_copilot_dn");
}

ZiboPAP3Profile::~ZiboPAP3Profile() { stop(); }

// -----------------------------------------------------------------------------
// Interface
// -----------------------------------------------------------------------------
bool ZiboPAP3Profile::isEligible() const { return _drZiboCheck != nullptr; }

void ZiboPAP3Profile::start(StateCallback onChanged) {
    if (_running) return;
    _cb = std::move(onChanged);
    _running = true;
}

void ZiboPAP3Profile::stop() {
    if (!_running) return;
    _running = false;
}

pap3::aircraft::State ZiboPAP3Profile::current() const { return _state; }

void ZiboPAP3Profile::tick() {
    if (!_running) return;
    if (_pendingIntentApply && _haveIntent) {
        applyIntent();

        const bool fdCaptMatch = (dr_get_pos(_drFDCaptPos) == (_intent.fd_capt_on ? FD_ON_POS : FD_OFF_POS));
        const bool fdFoMatch   = (dr_get_pos(_drFDFoPos)   == (_intent.fd_fo_on   ? FD_ON_POS : FD_OFF_POS));
        const bool apMatch     = (dr_get_pos(_drApDiscPos) == (_intent.ap_engaged ? AP_ON_POS : AP_OFF_POS));

        if (fdCaptMatch && fdFoMatch && apMatch) {
            _pendingIntentApply = false;
        }
    }

    poll(); // lecture sim -> State -> callback device
}

// -----------------------------------------------------------------------------
// Poll sim -> state (NE FORCE RIEN VERS LE SIM)
// -----------------------------------------------------------------------------
void ZiboPAP3Profile::poll() {
    if (_drSpd)     _state.spd     = XPLMGetDataf(_drSpd);
    if (_drHdg)     _state.hdg     = XPLMGetDatai(_drHdg);
    if (_drAlt)     _state.alt     = XPLMGetDatai(_drAlt);
    if (_drVvi)     _state.vvi     = XPLMGetDataf(_drVvi);
    if (_drVviShow) _state.vviVisible = XPLMGetDataf(_drVviShow) > 0.5f;
    if (_drCrsCapt) _state.crsCapt = XPLMGetDatai(_drCrsCapt);
    if (_drCrsFo)   _state.crsFo   = XPLMGetDatai(_drCrsFo);
    if (_drATArmPos) _state.atArmOn = (dr_get_pos(_drATArmPos) == AT_ON_POS);

    if (_drMcpBrightnessArr) {
        float v = 0.0f; XPLMGetDatavf(_drMcpBrightnessArr, &v, 15, 1);
        _state.mcpBrightness = std::clamp(v, 0.0f, 1.0f);
    }
    if (_drCockpitLightsArr) {
        float v = 0.0f; XPLMGetDatavf(_drCockpitLightsArr, &v, 0, 1);
        _state.cockpitLights  = std::clamp(v, 0.0f, 1.0f);
        _state.ledsBrightness = std::max(v, 0.60f);
    }

    if (_drDigitA) _state.digitA = XPLMGetDataf(_drDigitA) > 0.5f;
    if (_drDigitB) _state.digitB = XPLMGetDataf(_drDigitB) > 0.5f;

    _state.led.N1      = (_drLedN1     && XPLMGetDataf(_drLedN1)     > 0.5f);
    _state.led.SPEED   = (_drLedSpd    && XPLMGetDataf(_drLedSpd)    > 0.5f);
    _state.led.VNAV    = (_drLedVnav   && XPLMGetDataf(_drLedVnav)   > 0.5f);
    _state.led.LVL_CHG = (_drLedLvlChg && XPLMGetDataf(_drLedLvlChg) > 0.5f);
    _state.led.HDG_SEL = (_drLedHdgSel && XPLMGetDataf(_drLedHdgSel) > 0.5f);
    _state.led.LNAV    = (_drLedLnav   && XPLMGetDataf(_drLedLnav)   > 0.5f);
    _state.led.VORLOC  = (_drLedVorLoc && XPLMGetDataf(_drLedVorLoc) > 0.5f);
    _state.led.APP     = (_drLedApp    && XPLMGetDataf(_drLedApp)    > 0.5f);
    _state.led.ALT_HLD = (_drLedAltHld && XPLMGetDataf(_drLedAltHld) > 0.5f);
    _state.led.V_S     = (_drLedVs     && XPLMGetDataf(_drLedVs)     > 0.5f);
    _state.led.CMD_A   = (_drLedCmdA   && XPLMGetDataf(_drLedCmdA)   > 0.5f);
    _state.led.CWS_A   = (_drLedCwsA   && XPLMGetDataf(_drLedCwsA)   > 0.5f);
    _state.led.CMD_B   = (_drLedCmdB   && XPLMGetDataf(_drLedCmdB)   > 0.5f);
    _state.led.CWS_B   = (_drLedCwsB   && XPLMGetDataf(_drLedCwsB)   > 0.5f);
    _state.led.AT_ARM  = (_drLedAtArm  && XPLMGetDataf(_drLedAtArm)  > 0.5f);
    _state.led.MA_CAPT = (_drLedMaCapt && XPLMGetDataf(_drLedMaCapt) > 0.5f);
    _state.led.MA_FO   = (_drLedMaFo   && XPLMGetDataf(_drLedMaFo)   > 0.5f);

    if (_cb) _cb(_state);
}

// -----------------------------------------------------------------------------
// Command helpers
// -----------------------------------------------------------------------------
void ZiboPAP3Profile::execOnce(XPLMCommandRef cmd) { if (cmd) XPLMCommandOnce(cmd); }

void ZiboPAP3Profile::repeatCmd(XPLMCommandRef inc, XPLMCommandRef dec, int8_t delta, int stepPerTick) {
    if (delta == 0) return;
    const bool up  = (delta > 0);
    const int  reps = std::max(1, static_cast<int>(std::abs(static_cast<int>(delta))) * std::max(1, stepPerTick));
    XPLMCommandRef cmd = up ? inc : dec;
    if (!cmd) return;
    for (int i = 0; i < reps; ++i) XPLMCommandOnce(cmd);
}

bool ZiboPAP3Profile::debounce(float& lastTs, float minDeltaSec) const {
    const float now = XPLMGetElapsedTime();
    if (now - lastTs < minDeltaSec) return false;
    lastTs = now;
    return true;
}

// ---------- toggler bas-niveau ----------
void ZiboPAP3Profile::toggleToPosIfNeeded(XPLMDataRef posRef,
                                          int desiredPos,
                                          XPLMCommandRef toggleCmd,
                                          float& guardTs,
                                          int& lastCommandedPos) {
    if (!posRef || !toggleCmd) return;
    const int simPos = dr_get_pos(posRef);

    if (simPos == desiredPos) {
        guardTs = 0.f;
        lastCommandedPos = desiredPos;
        return;
    }

    if (desiredPos != lastCommandedPos) {
        guardTs = 0.f; // nouvelle demande: ne pas ralentir la première commande
    }

    const float now = XPLMGetElapsedTime();
    if (guardTs > 0.f && (now - guardTs) < 0.20f) return; // éviter de spammer si le sim n'a pas encore suivi

    guardTs = now;
    lastCommandedPos = desiredPos;
    debug_force("[PAP3][TOGGLE] posRef=%p simPos=%d desired=%d -> toggle cmd\n",
                static_cast<void*>(posRef), simPos, desiredPos);
    XPLMCommandOnce(toggleCmd);
}

// -----------------------------------------------------------------------------
// Input hooks -> update Intent -> applyIntent()
// -----------------------------------------------------------------------------
void ZiboPAP3Profile::onButton(std::uint8_t off, std::uint8_t mask, bool pressed) {
    // --- BANK ANGLE (0x05) ---
    if (off == 0x05 && pressed) {
        int target = -1;
        switch (mask) {
            case 0x02: target = 0; break; // 10°
            case 0x04: target = 1; break; // 15°
            case 0x08: target = 2; break; // 20°
            case 0x10: target = 3; break; // 25°
            case 0x20: target = 4; break; // 30°
            default: break;
        }
        if (target >= 0) { nudgeBankAngleTo(target); return; }
    }

    // -------- F/D CAPT ON (0x04/0x08) --------
    if (off == 0x04 && mask == 0x08) {
        
        if (!pressed) return;
        debug_force("F/D CAPT ON\n");
        _intent.fd_capt_on = true;
        _haveIntent = true;
    toggleToPosIfNeeded(_drFDCaptPos, FD_ON_POS, _cmdFDCaptToggle, _guardFdCaptTs, _lastFdCaptCmdPos);
        return;
    }

    // -------- F/D CAPT OFF (0x04/0x10) --------
    if (off == 0x04 && mask == 0x10) {
        
        if (!pressed) return;
        debug_force("F/D CAPT OFF\n");
        _intent.fd_capt_on = false;
        _haveIntent = true;
    toggleToPosIfNeeded(_drFDCaptPos, FD_OFF_POS, _cmdFDCaptToggle, _guardFdCaptTs, _lastFdCaptCmdPos);
        return;
    }

    // -------- F/D FO ON (0x04/0x20) --------
    if (off == 0x04 && mask == 0x20) {
        
        if (!pressed) return;
        debug_force("F/D FO ON\n");
        _intent.fd_fo_on = true;
        _haveIntent = true;
    toggleToPosIfNeeded(_drFDFoPos, FD_ON_POS, _cmdFDFoToggle, _guardFdFoTs, _lastFdFoCmdPos);
        return;
    }

    // -------- F/D FO OFF (0x04/0x40) --------
    if (off == 0x04 && mask == 0x40) {
        
        if (!pressed) return;
        debug_force("F/D FO OFF\n");
        _intent.fd_fo_on = false;
        _haveIntent = true;
    toggleToPosIfNeeded(_drFDFoPos, FD_OFF_POS, _cmdFDFoToggle, _guardFdFoTs, _lastFdFoCmdPos);
        return;
    }

    // -------- A/T ARM ON (0x06/0x01) --------
    if (off == 0x06 && mask == 0x01) {
        
        if (!pressed) return;
        debug_force("A/T ARM ON\n");
        _intent.at_arm_on = true;
        _haveIntent = true;
        if (_device) {
            _device->setAtHardwareIntent(true);
        }
    const int desiredPos = AT_ON_POS;
        if (desiredPos != _lastAtCmdPos) {
            _lastAtToggleTime = 0.0f;
        }
        if (debounce(_lastAtToggleTime, 0.15f)) {
            toggleToPosIfNeeded(_drATArmPos, desiredPos, _cmdATArmToggle, _guardAtTs, _lastAtCmdPos);
        }
        return;
    }

    // -------- A/T ARM OFF (0x06/0x02) --------
    if (off == 0x06 && mask == 0x02) {
        
        if (!pressed) return;
        debug_force("A/T ARM OFF\n");
        _intent.at_arm_on = false;
        _haveIntent = true;
        if (_device) {
            _device->setAtHardwareIntent(false);
        }
    const int desiredPos = AT_OFF_POS;
        if (desiredPos != _lastAtCmdPos) {
            _lastAtToggleTime = 0.0f;
        }
        if (debounce(_lastAtToggleTime, 0.15f)) {
            toggleToPosIfNeeded(_drATArmPos, desiredPos, _cmdATArmToggle, _guardAtTs, _lastAtCmdPos);
        }
        return;
    }

    // -------- A/P DISENGAGE ON (0x04/0x80) --------
    if (off == 0x04 && mask == 0x80) {
        
        if (!pressed) return;
        debug_force("A/P DISENGAGE ON\n");
        _intent.ap_engaged = true;
        _haveIntent = true;
    const int desiredPos = AP_ON_POS;
        if (desiredPos != _lastApDiscCmdPos) {
            _lastApDiscToggleTime = 0.0f;
        }
        if (debounce(_lastApDiscToggleTime, 0.15f)) {
            toggleToPosIfNeeded(_drApDiscPos, desiredPos, _cmdApDiscToggle, _guardApDiscTs, _lastApDiscCmdPos);
        }
        return;
    }

    // -------- A/P DISENGAGE OFF (0x05/0x01) --------
    if (off == 0x05 && mask == 0x01) {
        
        if (!pressed) return;
        debug_force("A/P DISENGAGE OFF\n");
        _intent.ap_engaged = false;
        _haveIntent = true;
    const int desiredPos = AP_OFF_POS;
        if (desiredPos != _lastApDiscCmdPos) {
            _lastApDiscToggleTime = 0.0f;
        }
        if (debounce(_lastApDiscToggleTime, 0.15f)) {
            toggleToPosIfNeeded(_drApDiscPos, desiredPos, _cmdApDiscToggle, _guardApDiscTs, _lastApDiscCmdPos);
        }
        return;
    }

    // -------- Boutons momentary restants --------
    for (const auto& b : _btns) {
        if (b.off == off && b.mask == mask) {
            execOnce(pressed ? b.press : b.release);
            return;
        }
    }
}

void ZiboPAP3Profile::onEncoderDelta(std::uint8_t posOff, std::int8_t delta) {
    for (const auto& e : _encs) {
        if (e.posOff == posOff) {
            repeatCmd(e.inc, e.dec, delta, e.stepPerTick);
            return;
        }
    }
}

// -----------------------------------------------------------------------------
// Power gating
// -----------------------------------------------------------------------------
uint8_t ZiboPAP3Profile::mcpPowerMask() const {
    uint8_t mask = 0;
    if (dr_is_on(_drHasApPower)) mask |= 0x01; // AP has power
    if (dr_is_on(_drDcBus1))     mask |= 0x02; // DC bus 1
    if (dr_is_on(_drDcBus2))     mask |= 0x04; // DC bus 2
    return mask;
}
bool ZiboPAP3Profile::mcpHasPower() const { return (mcpPowerMask() & 0x01) != 0; }

// -----------------------------------------------------------------------------
// Boot syncs (cohérents et simples)
// -----------------------------------------------------------------------------
void ZiboPAP3Profile::seedFromSim() {
    // F/D: ON => pos 1 ; OFF => pos 0
    _intent.fd_capt_on = (dr_get_pos(_drFDCaptPos) == FD_ON_POS);
    _intent.fd_fo_on   = (dr_get_pos(_drFDFoPos)   == FD_ON_POS);
    // A/T: ARMED => pos 1
    _intent.at_arm_on  = (dr_get_pos(_drATArmPos)  == AT_ON_POS);
    if (_drATArmPos) {
        _state.atArmOn = (dr_get_pos(_drATArmPos) == AT_ON_POS);
    }
    // A/P: ENGAGED => pos 0
    _intent.ap_engaged = (dr_get_pos(_drApDiscPos) == AP_ON_POS);
    _lastFdCaptCmdPos  = dr_get_pos(_drFDCaptPos);
    _lastFdFoCmdPos    = dr_get_pos(_drFDFoPos);
    _lastAtCmdPos      = dr_get_pos(_drATArmPos);
    _lastApDiscCmdPos  = dr_get_pos(_drApDiscPos);
    _haveIntent = true;
    _pendingIntentApply = true;
    if (_device) {
        _device->setAtHardwareIntent(_intent.at_arm_on);
    }
}
void ZiboPAP3Profile::seedFromRaw(const std::uint8_t* r, int len) {
    // F/D flags (On/Off lignes séparées)
    const bool fdCaptOn  = bit(r, len, 0x04, 0x08);
    const bool fdCaptOff = bit(r, len, 0x04, 0x10);
    if      (fdCaptOn)  _intent.fd_capt_on = true;
    else if (fdCaptOff) _intent.fd_capt_on = false;
    else                _intent.fd_capt_on = (dr_get_pos(_drFDCaptPos) == FD_ON_POS);

    const bool fdFoOn  = bit(r, len, 0x04, 0x20);
    const bool fdFoOff = bit(r, len, 0x04, 0x40);
    if      (fdFoOn)  _intent.fd_fo_on = true;
    else if (fdFoOff) _intent.fd_fo_on = false;
    else              _intent.fd_fo_on = (dr_get_pos(_drFDFoPos) == FD_ON_POS);

    // A/T: 0x06/0x01 -> pos 1 ; 0x06/0x02 -> pos 0 ; sinon conserve sim
    const bool atOn  = bit(r, len, 0x06, 0x01);
    const bool atOff = bit(r, len, 0x06, 0x02);
    if      (atOn)  _intent.at_arm_on = true;
    else if (atOff) _intent.at_arm_on = false;
    else            _intent.at_arm_on = (dr_get_pos(_drATArmPos) == AT_ON_POS);

    // A/P: ENGAGED 0x04/0x80 -> pos 0 ; DISENGAGED 0x05/0x01 -> pos 1 ; sinon conserve sim
    const bool apEng = bit(r, len, 0x04, 0x80);
    const bool apDis = bit(r, len, 0x05, 0x01);
    if      (apEng) _intent.ap_engaged = true;
    else if (apDis) _intent.ap_engaged = false;
    else            _intent.ap_engaged = (dr_get_pos(_drApDiscPos) == AP_ON_POS);

    _lastFdCaptCmdPos = dr_get_pos(_drFDCaptPos);
    _lastFdFoCmdPos   = dr_get_pos(_drFDFoPos);
    const int simAtPos = dr_get_pos(_drATArmPos);
    _lastAtCmdPos     = simAtPos;
    _state.atArmOn    = (simAtPos == AT_ON_POS);
    _lastApDiscCmdPos = dr_get_pos(_drApDiscPos);

    _haveIntent = true;
    _pendingIntentApply = true;
    if (_device) {
        _device->setAtHardwareIntent(_intent.at_arm_on);
    }
}
void ZiboPAP3Profile::applyIntent() {
    if (!_haveIntent) return;

    // F/D
    toggleToPosIfNeeded(_drFDCaptPos, _intent.fd_capt_on ? FD_ON_POS : FD_OFF_POS, _cmdFDCaptToggle, _guardFdCaptTs, _lastFdCaptCmdPos);
    toggleToPosIfNeeded(_drFDFoPos,   _intent.fd_fo_on   ? FD_ON_POS : FD_OFF_POS, _cmdFDFoToggle,   _guardFdFoTs, _lastFdFoCmdPos);

    // A/T: always request sim to match intent; simulator may refuse and we'll react elsewhere
    toggleToPosIfNeeded(_drATArmPos, _intent.at_arm_on ? AT_ON_POS : AT_OFF_POS, _cmdATArmToggle, _guardAtTs, _lastAtCmdPos);
    if (_device) {
        _device->setAtHardwareIntent(_intent.at_arm_on);
    }

    const int desiredApDiscPos = _intent.ap_engaged ? AP_ON_POS : AP_OFF_POS;
    toggleToPosIfNeeded(_drApDiscPos, desiredApDiscPos, _cmdApDiscToggle, _guardApDiscTs, _lastApDiscCmdPos);
}

void ZiboPAP3Profile::syncSimToHardware() {
    seedFromSim();
    applyIntent();
}

void ZiboPAP3Profile::syncSimToHardwareFromRaw(const std::uint8_t* r, int len) {
    if (r && len > 0) seedFromRaw(r, len);
    else              seedFromSim();
    applyIntent();

    // Bank angle snapshot (optionnel)
    int targetBank = -1;
    if      (bit(r, len, 0x05, 0x02)) targetBank = 0; // 10°
    else if (bit(r, len, 0x05, 0x04)) targetBank = 1; // 15°
    else if (bit(r, len, 0x05, 0x08)) targetBank = 2; // 20°
    else if (bit(r, len, 0x05, 0x10)) targetBank = 3; // 25°
    else if (bit(r, len, 0x05, 0x20)) targetBank = 4; // 30°
    if (targetBank >= 0) nudgeBankAngleTo(targetBank);

    _lastAtToggleTime = _lastApDiscToggleTime = 0.0f;
}

// -----------------------------------------------------------------------------
// Bank angle
// -----------------------------------------------------------------------------
int ZiboPAP3Profile::readBankIndex() const {
    if (!_drBankIdx) return -1;
    const XPLMDataTypeID ty = XPLMGetDataRefTypes(_drBankIdx);
    if (ty & xplmType_Int)    return std::clamp(XPLMGetDatai(_drBankIdx), 0, 4);
    if (ty & xplmType_Float)  return std::clamp((int)std::lround(XPLMGetDataf(_drBankIdx)), 0, 4);
    if (ty & xplmType_Double) return std::clamp((int)std::lround(XPLMGetDatad(_drBankIdx)), 0, 4);
    return -1;
}

void ZiboPAP3Profile::nudgeBankAngleTo(int target) {
    if (target < 0 || target > 4) return;
    if (!_cmdBankUp && !_cmdBankDn) return;

    int cur = readBankIndex();
    if (cur < 0) return;

    const int maxSteps = 10;
    int steps = 0;
    while (cur != target && steps++ < maxSteps) {
        const bool up = (target > cur);
        XPLMCommandRef cmd = up ? _cmdBankUp : _cmdBankDn;
        if (!cmd) break;
        XPLMCommandOnce(cmd);

        int next = readBankIndex();
        if (next == cur) {
            XPLMCommandOnce(cmd);
            next = readBankIndex();
        }
        cur = next;
    }
}

} // namespace pap3::aircraft