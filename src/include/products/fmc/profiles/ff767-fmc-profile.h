#ifndef FF767_FMC_PROFILE_H
#define FF767_FMC_PROFILE_H

#include "fmc-aircraft-profile.h"
#include <regex>

class FlightFactor767FMCProfile : public FMCAircraftProfile {
private:
    static std::vector<std::string> datarefsList;
    static std::vector<FMCButtonDef> buttonsList;
    static std::map<char, int> colors;
        
public:
    FlightFactor767FMCProfile(ProductFMC *product);
    virtual ~FlightFactor767FMCProfile();

    static bool IsEligible();

    static constexpr uint16_t DataLength = 14 * 24; // 336 letters (14 lines x 24 chars)
    const std::vector<std::string>& displayDatarefs() const override;
    const std::vector<FMCButtonDef>& buttonDefs() const override;
    const std::map<char, FMCTextColor>& colorMap() const override;
    void mapCharacter(std::vector<uint8_t> *buffer, uint8_t character, bool isFontSmall) override;
    void updatePage(std::vector<std::vector<char>>& page) override;
    void buttonPressed(const FMCButtonDef *button, XPLMCommandPhase phase) override;
};

#endif
