#pragma once

#include <memory>

#include "poke/status.hpp"
#include "poke/types.hpp"

namespace poke::detail {

// Pure validation shared by connect() and offline configuration regression tests.
Status validateAuxiliaryConfig(const AuxiliaryConfig& config);

class AuxiliaryController {
public:
    explicit AuxiliaryController(AuxiliaryConfig config);
    ~AuxiliaryController();

    AuxiliaryController(const AuxiliaryController&) = delete;
    AuxiliaryController& operator=(const AuxiliaryController&) = delete;

    Status connect();
    Status enable();
    Status startMotion();
    void requestStop();
    Status waitMotion();
    Status stopMotion();
    Status setChassisEnabled(bool enabled);
    Status setChassisWheelVelocities(const std::array<double, 4>& velocities_rad_s);
    Status stopChassis();
    Status refreshChassisFeedback();
    Status setLiftEnabled(bool enabled);
    Status setLiftVelocity(double velocity_counts_s);
    Status setLiftPositionMm(double position_mm, double profile_velocity_mm_s,
                             bool calibration_jog);
    Status setLiftCurrentPositionAsZero();
    Status stopLift();
    Status refreshLiftFeedback();
    Status disable();
    void close();
    AuxiliaryState state() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace poke::detail
