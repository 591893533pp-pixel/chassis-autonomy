#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace poke {

struct ChassisTwist {
    double vx_m_s{0.0};
    double vy_m_s{0.0};
    double omega_z_rad_s{0.0};
};

struct XDriveWheelSolution {
    std::array<double, 4> unscaled_rad_s{};
    std::array<double, 4> wheel_rad_s{};
    ChassisTwist applied_twist{};
    double scale{1.0};
};

// Four identical 45-degree omni wheels in the machine's empirically verified
// motor order/sign convention:
//   1: -forward +left +yaw, 2: +forward +left +yaw,
//   3: +forward -left +yaw, 4: -forward -left +yaw.
// The forward polarity was corrected on the deployed chassis on 2026-08-30;
// logical +vx now drives the physical robot forward. The lateral polarity was
// corrected on 2026-08-31; logical +vy now drives the physical robot left.
// wheel_center_radius_m is the radial chassis-center -> wheel-center distance.
class XDriveKinematics {
public:
    XDriveKinematics(double wheel_radius_m,
                     double wheel_center_radius_m,
                     double maximum_wheel_speed_rad_s)
        : wheel_radius_m_(wheel_radius_m),
          wheel_center_radius_m_(wheel_center_radius_m),
          maximum_wheel_speed_rad_s_(maximum_wheel_speed_rad_s) {
        if (!std::isfinite(wheel_radius_m_) || wheel_radius_m_ <= 0.0 ||
            !std::isfinite(wheel_center_radius_m_) ||
            wheel_center_radius_m_ <= 0.0 ||
            !std::isfinite(maximum_wheel_speed_rad_s_) ||
            maximum_wheel_speed_rad_s_ <= 0.0)
            throw std::runtime_error("invalid X-drive geometry or wheel-speed cap");
    }

    XDriveWheelSolution inverse(const ChassisTwist& requested) const {
        if (!std::isfinite(requested.vx_m_s) ||
            !std::isfinite(requested.vy_m_s) ||
            !std::isfinite(requested.omega_z_rad_s))
            throw std::runtime_error("non-finite chassis body twist");

        constexpr double kInvSqrt2 = 0.70710678118654752440;
        const double translation_scale = kInvSqrt2 / wheel_radius_m_;
        const double yaw_scale =
            wheel_center_radius_m_ / wheel_radius_m_;
        XDriveWheelSolution result;
        result.unscaled_rad_s = {{
            translation_scale * (-requested.vx_m_s + requested.vy_m_s) +
                yaw_scale * requested.omega_z_rad_s,
            translation_scale * (requested.vx_m_s + requested.vy_m_s) +
                yaw_scale * requested.omega_z_rad_s,
            translation_scale * (requested.vx_m_s - requested.vy_m_s) +
                yaw_scale * requested.omega_z_rad_s,
            translation_scale * (-requested.vx_m_s - requested.vy_m_s) +
                yaw_scale * requested.omega_z_rad_s,
        }};
        double peak = 0.0;
        for (const double value : result.unscaled_rad_s)
            peak = std::max(peak, std::abs(value));
        result.scale = peak > maximum_wheel_speed_rad_s_
            ? maximum_wheel_speed_rad_s_ / peak : 1.0;
        for (std::size_t index = 0; index < result.wheel_rad_s.size(); ++index)
            result.wheel_rad_s[index] =
                result.scale * result.unscaled_rad_s[index];
        result.applied_twist = {
            result.scale * requested.vx_m_s,
            result.scale * requested.vy_m_s,
            result.scale * requested.omega_z_rad_s,
        };
        return result;
    }

    ChassisTwist forward(const std::array<double, 4>& wheel_rad_s) const {
        for (const double value : wheel_rad_s)
            if (!std::isfinite(value))
                throw std::runtime_error("non-finite X-drive wheel feedback");
        constexpr double kInv2Sqrt2 = 0.35355339059327376220;
        return {
            wheel_radius_m_ * kInv2Sqrt2 *
                (-wheel_rad_s[0] + wheel_rad_s[1] +
                 wheel_rad_s[2] - wheel_rad_s[3]),
            wheel_radius_m_ * kInv2Sqrt2 *
                (wheel_rad_s[0] + wheel_rad_s[1] -
                 wheel_rad_s[2] - wheel_rad_s[3]),
            wheel_radius_m_ /
                (4.0 * wheel_center_radius_m_) *
                (wheel_rad_s[0] + wheel_rad_s[1] +
                 wheel_rad_s[2] + wheel_rad_s[3]),
        };
    }

private:
    double wheel_radius_m_{0.0};
    double wheel_center_radius_m_{0.0};
    double maximum_wheel_speed_rad_s_{0.0};
};

}  // namespace poke
