// ProductFMC.h
#ifndef PRODUCT_FMC_H
#define PRODUCT_FMC_H

#include "usbdevice.h"
#include "fmc-aircraft-profile.h"
#include <map>
#include <set>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <chrono>

class ProductFMC: public USBDevice {
private:

    // --- Worker I/O ---
    struct IoCmd {
        enum Type : uint8_t {
            UsbBuffer,        // envoi d’un gros buffer LCD (déjà packé 0xF2..)
            SetLedBrightness, // un LED + brightness
            ClearScreen,      // 16 lignes blanches
            ShowBackground,   // image de fond
            UploadFont        // liste de chunks font (rare)
        } type;
        uint8_t a = 0;    // led id, etc.
        uint8_t b = 0;    // brightness, etc.
        std::vector<uint8_t> payload;                   // pour UsbBuffer (multi de 63 -> le worker segmente)
        std::vector<std::vector<uint8_t>> payloadChunks;// pour UploadFont
    };

    std::thread              _ioThread{};
    std::atomic<bool>        _ioRunning{false};
    std::mutex               _ioMx;
    std::condition_variable  _ioCv;
    std::deque<IoCmd>        _ioQueue;

    // Coalescing / dernier état vraiment envoyé (pour éviter le spam)
    std::vector<uint8_t>     _sentFrame;   // dernier buffer 0xF2… envoyé
    double                   _lastUsbTs{0.0};
    double                   _minUsbPeriod{1.0/25.0}; // ~25 Hz comme avant

    // Anti-concurrence sur writeData si besoin
    std::mutex               _usbWriteMx;

    // Helpers queue
    inline void qEnqueue(IoCmd&& c) {
        { std::lock_guard<std::mutex> lk(_ioMx); _ioQueue.emplace_back(std::move(c)); }
        _ioCv.notify_one();
    }
    inline void qUsbBuffer(const std::vector<uint8_t>& buf) {
        IoCmd c; c.type = IoCmd::UsbBuffer; c.payload = buf; qEnqueue(std::move(c));
    }
    inline void qLed(FMCLed led, uint8_t b) {
        IoCmd c; c.type = IoCmd::SetLedBrightness; c.a = static_cast<uint8_t>(led); c.b = b; qEnqueue(std::move(c));
    }
    inline void qClear() {
        IoCmd c; c.type = IoCmd::ClearScreen; qEnqueue(std::move(c));
    }
    inline void qBackground(FMCBackgroundVariant v, const std::vector<uint8_t>& frame) {
        IoCmd c; c.type = IoCmd::ShowBackground; c.payload = frame; qEnqueue(std::move(c));
    }
    inline void qUploadFont(const std::vector<std::vector<unsigned char>>& font) {
        IoCmd c; c.type = IoCmd::UploadFont;
        c.payloadChunks.reserve(font.size());
        for (auto& ch : font) c.payloadChunks.emplace_back(ch.begin(), ch.end());
        qEnqueue(std::move(c));
    }

    void ioThreadMain();  // implémenté dans .cpp

    // --- (déjà ajoutés précédemment) anti-spam par instance ---
    std::vector<uint8_t> _lastBuf; // diff local draw()
    double _lastPush = 0.0;

    // --- état fonts (déjà proposé) ---
    uint32_t _fontCrc = 0;
    double   _fontLastTs = 0.0;
    bool     _fontEverSent = false;

    FMCAircraftProfile *profile;
    std::vector<std::vector<char>> page;
    int lastUpdateCycle;
    std::set<int> pressedButtonIndices;
    uint64_t lastButtonStateLo;
    uint32_t lastButtonStateHi;
    
    void updatePage();
    void draw(const std::vector<std::vector<char>> *pagePtr = nullptr);
    std::pair<uint8_t, uint8_t> dataFromColFont(char color, bool fontSmall = false);
    
    void setProfileForCurrentAircraft();

public:
    ProductFMC(HIDDeviceHandle hidDevice, uint16_t vendorId, uint16_t productId, std::string vendorName, std::string productName, FMCHardwareType hardwareType, unsigned char identifierByte);
    ~ProductFMC();
    
    static constexpr unsigned int PageLines = 14; // Header + 6 * label + 6 * cont + textbox
    static constexpr unsigned int PageCharsPerLine = 24;
    static constexpr unsigned int PageBytesPerChar = 3;
    static constexpr unsigned int PageBytesPerLine = PageCharsPerLine * PageBytesPerChar;
    FMCHardwareType hardwareType;
    const unsigned char identifierByte;
    bool fontUpdatingEnabled;

    inline void allowFontUpdateOnce() {
        fontUpdatingEnabled = true;
    }

    const char* classIdentifier() override;
    bool connect() override;
    void disconnect() override;
    void unloadProfile();
    void update() override;
    void didReceiveData(int reportId, uint8_t *report, int reportLength) override;
    void writeLineToPage(std::vector<std::vector<char>>& page, int line, int pos, const std::string &text, char color, bool fontSmall = false);
    void setFont(std::vector<std::vector<unsigned char>> font);
    
    void setAllLedsEnabled(bool enable);
    void setLedBrightness(FMCLed led, uint8_t brightness);
    
    void clearDisplay();
    void showBackground(FMCBackgroundVariant variant);
};

#endif
