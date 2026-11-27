#pragma once

#include "poke/chassis_kinematics.hpp"

#include <cmath>

namespace poke {

struct ChassisPose2d {
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
};

inline bool integrateMeasuredBodyTwist(
    ChassisPose2d& pose, const ChassisTwist& measured, double dt) {
    if (!(std::isfinite(dt) && dt >= 0.0) ||
        !std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
        !std::isfinite(pose.yaw_rad) ||
        !std::isfinite(measured.vx_m_s) ||
        !std::isfinite(measured.vy_m_s) ||
        !std::isfinite(measured.omega_z_rad_s))
        return false;

    // Midpoint SE(2) integration is adequate at the supported control rates
    // (50-500 Hz; accuracy only improves as dt shrinks) and avoids treating
    // body-frame vx/vy as fixed world-frame axes while the chassis is
    // rotating.
    const double yaw_step = measured.omega_z_rad_s * dt;
    const double midpoint_yaw = pose.yaw_rad + 0.5 * yaw_step;
    const double cosine = std::cos(midpoint_yaw);
    const double sine = std::sin(midpoint_yaw);
    pose.x_m +=
        (cosine * measured.vx_m_s - sine * measured.vy_m_s) * dt;
    pose.y_m +=
        (sine * measured.vx_m_s + cosine * measured.vy_m_s) * dt;
    pose.yaw_rad += yaw_step;
    return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
           std::isfinite(pose.yaw_rad);
}

}  // namespace poke
