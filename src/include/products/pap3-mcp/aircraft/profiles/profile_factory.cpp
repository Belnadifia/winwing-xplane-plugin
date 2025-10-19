#include "profile_factory.h"
#include "zibo_profile.h"

namespace pap3::aircraft {

std::unique_ptr<PAP3AircraftProfile> ProfileFactory::detect() {
    // Essayer Zibo d’abord
    {
        auto zibo = std::make_unique<ZiboPAP3Profile>();
        if (zibo->isEligible()) {
            // conversion dérivée -> base (héritage visible ici)
            return std::unique_ptr<PAP3AircraftProfile>(std::move(zibo));
        }
    }
    return nullptr;
}

} // namespace pap3::aircraft