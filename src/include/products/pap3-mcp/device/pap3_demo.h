#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <random>

namespace pap3::device {

class PAP3Device; // forward declaration

class PAP3Demo {
public:
    explicit PAP3Demo(PAP3Device* dev);
    ~PAP3Demo();

    // Démarre/arrête la démo. Si ledIds est vide, on utilise [0..15].
    void start(const std::vector<std::uint8_t>& ledIds,
               float lcdFps = 12.0f,
               float ledFps = 18.0f,
               float hbFps  = 24.0f);
    void stop();
    bool running() const noexcept { return _running; }

    // Réglages runtime
    void setText(const std::string& t);
    void setFps(float lcdFps, float ledFps, float hbFps);

    // Trampoline public utilisé par XPLMRegisterFlightLoopCallback
    static float FlightLoopThunk(float inElapsedSinceLastCall, float, int, void* refcon);

private:
    // Tick appelé par le flight loop
    float updateTick(float elapsedSinceLastCall);

    // --- LCD marquee ---
    void lcdStepMarquee();
    std::vector<std::uint8_t> buildLcdPayloadForWindow(int startCol) const;

    // --- LED pattern ---
    void ledStepPattern();

    // --- Heartbeat (backlight + LCD only) ---
    void heartbeatStep(double dt);

    // --- Glyphs 7-segments (A..G) utilisés pour la démo ---
    static std::uint8_t segMaskForChar(char c);

private:
    PAP3Device*               _dev { nullptr }; // non-owning
    bool                      _running { false };

    // Scheduling
    double                    _accumLcd { 0.0 }, _accumLed { 0.0 };
    double                    _dtLcd { 1.0/12.0 }, _dtLed { 1.0/18.0 };

    // LCD marquee state
    std::string               _text { "  HELLO  PAP3  " }; // avec padding
    int                       _col { 0 };
    int                       _lineLen { 22 };             // 22 digits visibles (CAPT 3 + SPD 4 + HDG 3 + ALT 5 + V/S 4 + FO 3)
    std::vector<std::uint8_t> _lastLcdPayload;             // coalescing USB

    // LED pattern state
    std::vector<std::uint8_t> _ledIds;
    int                       _head { 0 };          // index courant (0..n-1)
    int                       _dir  { +1 };         // +1 vers la droite, -1 vers la gauche
    int                       _prevA { -1 }, _prevB { -1 }; // derniers index allumés
    std::mt19937              _rng;

    // Heartbeat state
    double                    _phase { 0.0 };
    double                    _hbErr { 0.0 };
    std::uint8_t              _hbMin { 40 };
    std::uint8_t              _hbMax { 220 };
};

} // namespace pap3::device