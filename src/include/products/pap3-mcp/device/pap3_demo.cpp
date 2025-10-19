#include "pap3_demo.h"
#include "pap3_device.h"
#include "../lcd/segments.h"

#include <XPLMProcessing.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <array>
#include <random>

using pap3::lcd::segments::Payload;
using pap3::lcd::segments::GroupOffsets;
using pap3::lcd::segments::clear;
using pap3::lcd::segments::setFlag;

// ------- Bits logiques (A..G) pour 7-seg --------
static constexpr std::uint8_t SEG_A = 1u << 0; // top
static constexpr std::uint8_t SEG_B = 1u << 1; // top-right
static constexpr std::uint8_t SEG_C = 1u << 2; // bottom-right
static constexpr std::uint8_t SEG_D = 1u << 3; // bottom
static constexpr std::uint8_t SEG_E = 1u << 4; // bottom-left
static constexpr std::uint8_t SEG_F = 1u << 5; // top-left
static constexpr std::uint8_t SEG_G = 1u << 6; // middle

namespace pap3::device {

// ---- trampoline flight loop -------------------------------------------------
static float PAP3Demo_FlightLoop(float inElapsedSinceLastCall, float, int, void* refcon) {
    auto* self = static_cast<PAP3Demo*>(refcon);
    if (!self) return 0.0f;
    return self->FlightLoopThunk(inElapsedSinceLastCall, 0.0f, 0, refcon);
}

// ---- Ctor / Dtor ------------------------------------------------------------
PAP3Demo::PAP3Demo(PAP3Device* dev)
: _dev(dev)
, _rng(std::random_device{}())
{}

PAP3Demo::~PAP3Demo() { stop(); }

// ---- API --------------------------------------------------------------------
void PAP3Demo::start(const std::vector<std::uint8_t>& ledIds, float lcdFps, float ledFps, float hbFps) {
    if (_running || !_dev) return;

    _ledIds = ledIds;
    if (_ledIds.size() < 2) {
        _ledIds.clear();
        for (uint8_t i=0;i<16;++i) _ledIds.push_back(i);
    }

    setFps(lcdFps, ledFps, hbFps);

    _accumLcd = _accumLed = 0.0;
    _col   = 0;
    _head  = 0;
    _dir   = +1;
    _prevA = _prevB = -1;
    _lastLcdPayload.clear();
    _hbErr = 0.0;

    // Le marquee utilise 22 “slots” visibles (voir mapping plus bas)
    _lineLen = 22;

    // Init LCD (ouvre la voie pour le tout premier payload)
    _dev->lcdInit();

    // Appel régulier (environ 25 Hz pour le LCD par défaut)
    XPLMRegisterFlightLoopCallback(PAP3Demo_FlightLoop, 0.04f, this);
    _running = true;
}

void PAP3Demo::stop() {
    if (!_running) return;

    // On éteint seulement ce qu'on a allumé
    if (_prevA < _ledIds.size()) _dev->setLed(_ledIds[_prevA], false);
    if (_prevB < _ledIds.size()) _dev->setLed(_ledIds[_prevB], false);

    XPLMUnregisterFlightLoopCallback(PAP3Demo_FlightLoop, this);
    _running = false;
}

void PAP3Demo::setText(const std::string& t) {
    _text = "  " + t + "  ";  // padding pour un scroll propre
    _col = 0;
    _lastLcdPayload.clear();
}

void PAP3Demo::setFps(float lcdFps, float ledFps, float hbFps) {
    _dtLcd = (lcdFps > 0.f) ? 1.0 / lcdFps : 1.0/12.0;
    _dtLed = (ledFps > 0.f) ? 1.0 / ledFps : 1.0/18.0;
}

// ---- Flight loop ------------------------------------------------------------
float PAP3Demo::FlightLoopThunk(float inElapsedSinceLastCall, float, int, void* refcon) {
    auto* self = static_cast<PAP3Demo*>(refcon);
    if (!self || !self->_running) return 0.0f;
    return self->updateTick(inElapsedSinceLastCall);
}

float PAP3Demo::updateTick(float elapsedSinceLastCall) {
    _accumLcd += elapsedSinceLastCall;
    _accumLed += elapsedSinceLastCall;

    if (_accumLcd >= _dtLcd) {
        _accumLcd -= _dtLcd;
        lcdStepMarquee();
    }
    if (_accumLed >= _dtLed) {
        _accumLed -= _dtLed;
        ledStepPattern();
    }

    heartbeatStep(elapsedSinceLastCall);

    // Re-appelle à la frame suivante
    return -1.0f;
}

// ---- LCD marquee ------------------------------------------------------------

// Rendu d'un “glyph mask” (A..G) vers les 7 positions d’un group offset
static inline void drawGlyph(const GroupOffsets& g, Payload& p, std::uint8_t flag, std::uint8_t mask) {
    if (mask & SEG_G) setFlag(p, g.mid,  flag, true);
    if (mask & SEG_F) setFlag(p, g.topL, flag, true);
    if (mask & SEG_E) setFlag(p, g.botL, flag, true);
    if (mask & SEG_D) setFlag(p, g.bot,  flag, true);
    if (mask & SEG_C) setFlag(p, g.botR, flag, true);
    if (mask & SEG_B) setFlag(p, g.topR, flag, true);
    if (mask & SEG_A) setFlag(p, g.top,  flag, true);
}

void PAP3Demo::lcdStepMarquee() {
    // Construit le payload (32 octets 0x19..0x38)
    std::vector<std::uint8_t> p = buildLcdPayloadForWindow(_col);

    // Coalescing (évite d'inonder l’USB si identique)
    if (p != _lastLcdPayload) {
        _dev->lcdSendPayload(p);
        _dev->lcdSendEmpty();
        _dev->lcdSendEmpty();
        _dev->lcdCommit();
        _lastLcdPayload = std::move(p);
    }

    // Avance la fenêtre
    ++_col;
    const int visible = _lineLen; // 22
    const int span = static_cast<int>(_text.size()) + visible;
    if (_col >= span) _col = 0;
}

std::vector<std::uint8_t> PAP3Demo::buildLcdPayloadForWindow(int startCol) const {
    // 32 bytes (0x19..0x38)
    Payload pay{}; clear(pay);

    // GroupOffsets (ordre des positions: Mid-TopL-BotL-Bot-BotR-TopR-Top)
    const GroupOffsets G1 { 0x1D, 0x21, 0x25, 0x29, 0x2D, 0x31, 0x35 }; // CPT_CRS + SPD
    const GroupOffsets G2 { 0x1E, 0x22, 0x26, 0x2A, 0x2E, 0x32, 0x36 }; // HDG + ALT_HI
    const GroupOffsets G3 { 0x1F, 0x23, 0x27, 0x2B, 0x2F, 0x33, 0x37 }; // ALT_LO + VSPD
    const GroupOffsets G4 { 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38 }; // FO_CRS

    // ---- DÉFINITION DES SLOTS VISIBLES (gauche -> droite à l’œil) ----
    // CRS_CAPT (3), IAS/MACH (4), HDG (3), ALT (5), V/S (4), CRS_FO (3) = 22
    struct Slot { GroupOffsets g; std::uint8_t flag; };

    const std::array<Slot, 22> kAllSlots = {
        // CRS_CAPT (G1: 0x80 0x40 0x20)
        Slot{G1, 0x80}, Slot{G1, 0x40}, Slot{G1, 0x20},
        // IAS/MACH / SPD (G1: 0x08 0x04 0x02 0x01)
        Slot{G1, 0x08}, Slot{G1, 0x04}, Slot{G1, 0x02}, Slot{G1, 0x01},
        // HDG (G2: 0x40 0x20 0x10)
        Slot{G2, 0x40}, Slot{G2, 0x20}, Slot{G2, 0x10},
        // ALT high (G2: 0x04 tens_kilo, 0x02 kilo, 0x01 hundreds)
        Slot{G2, 0x04}, Slot{G2, 0x02}, Slot{G2, 0x01},
        // ALT low (G3: 0x80 tens, 0x40 units)
        Slot{G3, 0x80}, Slot{G3, 0x40},
        // V/S (G3: 0x08 0x04 0x02 0x01)
        Slot{G3, 0x08}, Slot{G3, 0x04}, Slot{G3, 0x02}, Slot{G3, 0x01},
        // CRS_FO (G4: 0x40 0x20 0x10)
        Slot{G4, 0x40}, Slot{G4, 0x20}, Slot{G4, 0x10},
    };

    const int visible = static_cast<int>(kAllSlots.size()); // 22
    for (int i = 0; i < visible; ++i) {
        const int col = startCol + i - visible; // négatif = pré-roll à blanc
        const char ch = (col >= 0 && col < static_cast<int>(_text.size())) ? _text[col] : ' ';
        const std::uint8_t m = PAP3Demo::segMaskForChar(ch);
        if (!m) continue;

        const Slot& s = kAllSlots[static_cast<std::size_t>(i)];
        drawGlyph(s.g, pay, s.flag, m);
    }

    // (Optionnel) Exemples de labels/dots si tu veux les voir pendant la démo :
    // setFlag(pay, 0x36, 0x08, true); // HDG label (gauche)
    // setFlag(pay, 0x32, 0x08, true); // HDG label (droite)
    // setFlag(pay, 0x36, 0x80, true); // IAS label
    // setFlag(pay, 0x32, 0x80, true); // MACH L
    // setFlag(pay, 0x2E, 0x80, true); // MACH R

    return std::vector<std::uint8_t>(pay.begin(), pay.end());
}

// ---- LED pattern ------------------------------------------------------------
void PAP3Demo::ledStepPattern() {
    const int n = static_cast<int>(_ledIds.size());
    if (n <= 0) return;

    // 1) Éteindre proprement la paire précédente (si valide)
    if (_prevA >= 0 && _prevA < n) _dev->setLed(_ledIds[static_cast<std::size_t>(_prevA)], false);
    if (_prevB >= 0 && _prevB < n) _dev->setLed(_ledIds[static_cast<std::size_t>(_prevB)], false);

    // 2) Avancer la tête + gérer le rebond
    _head += _dir;
    if (_head >= n) {          // dépasse à droite
        _head = n - 1;
        _dir  = -1;
    } else if (_head < 0) {    // dépasse à gauche
        _head = 0;
        _dir  = +1;
    }

    // 3) Calculer une "queue" qui suit la tête à 1..3 positions
    //    (borne à n-1 pour les courtes bandes)
    std::uniform_int_distribution<int> jitter(-1, +1);
    int maxTrail = std::min(3, std::max(1, n - 1));
    int tailOff  = std::clamp(1 + jitter(_rng), 1, maxTrail);

    int tail = _head - (_dir * tailOff);
    tail = std::clamp(tail, 0, n - 1);

    // Si par hasard tail == head (petits n), décaler d’un cran opposé
    if (tail == _head) {
        tail = std::clamp(_head - _dir, 0, n - 1);
    }

    // 4) Allumer la nouvelle paire
    _dev->setLed(_ledIds[static_cast<std::size_t>(_head)], true);
    _dev->setLed(_ledIds[static_cast<std::size_t>(tail)], true);

    // 5) Mémoriser pour extinction au tick suivant
    _prevA = _head;
    _prevB = tail;
}

// ---- Heartbeat (backlight + LCD) -------------------------------------------
void PAP3Demo::heartbeatStep(double dt) {
    // 1) phase à chaque frame (respiration ~1.6 s → 0.6 Hz)
    const double freq = 0.6;
    _phase += dt * freq;
    if (_phase >= 1.0) _phase -= 1.0;

    // 2) courbe ease-in-out (cosine), plus “reposée” sur les maxima/minima
    const double ease = 0.5 - 0.5 * std::cos(2.0 * M_PI * _phase); // 0..1

    // 3) correction gamma perceptuelle (2.2 ≈ standard), évite l’effet “pompe”
    constexpr double gamma = 2.2;
    const double percept = std::pow(ease, 1.0 / gamma);            // 0..1

    // 4) mise à l’échelle + “temporal dithering” pour lisser la quantification 8-bit
    const double raw = _hbMin + percept * (_hbMax - _hbMin);       // 0..255 (double)
    const double withErr = raw + _hbErr;
    int v = (int)std::lround(withErr);
    v = std::clamp(v, 0, 255);
    _hbErr = withErr - v;  // conserve l’erreur pour la frame suivante

    const uint8_t val = static_cast<uint8_t>(v);
    _dev->setDimming(0, val);  // backlight
    _dev->setDimming(1, val);  // LCD
}

// ---- Glyphs 7-seg lisibles (“HELLO  PAP3”) ---------------------------------
std::uint8_t PAP3Demo::segMaskForChar(char c) {
    switch (c) {
        case 'H': case 'h': return (SEG_F|SEG_E|SEG_G|SEG_B|SEG_C);           // H
        case 'E': case 'e': return (SEG_A|SEG_F|SEG_G|SEG_E|SEG_D);           // E
        case 'L': case 'l': return (SEG_F|SEG_E|SEG_D);                       // L
        case 'O': case 'o': return (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F);     // O
        case 'P': case 'p': return (SEG_A|SEG_B|SEG_F|SEG_G|SEG_E);           // P
        case 'A': case 'a': return (SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G);     // A
        case '3':           return (SEG_A|SEG_B|SEG_C|SEG_D|SEG_G);           // 3
        case ' ':           return 0;
        default:            return 0;
    }
}

} // namespace pap3::device