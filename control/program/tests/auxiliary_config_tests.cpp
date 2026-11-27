#include <cstdlib>
#include <iostream>
#include <limits>
#include "auxiliary.hpp"

int main() {
    poke::AuxiliaryConfig cfg;
    cfg.serial = "offline-validation-only";
    cfg.enable_chassis = true;
    for (double speed : {0.8, 10.0, 25.0}) {
        cfg.chassis_speed_rad_s = speed;
        if (!poke::detail::validateAuxiliaryConfig(cfg)) {
            std::cerr << "valid wheel cap rejected: " << speed << '\n';
            return EXIT_FAILURE;
        }
    }
    for (double speed : {0.0, -1.0, 25.0001,
                         std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        cfg.chassis_speed_rad_s = speed;
        if (poke::detail::validateAuxiliaryConfig(cfg).code() != poke::ErrorCode::InvalidConfig) {
            std::cerr << "invalid wheel cap accepted\n";
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}
