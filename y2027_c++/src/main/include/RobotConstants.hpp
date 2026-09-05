#pragma once 

#include <subsystems/drivetrain/Constants.hpp>
#include <subsystems/intake/Constants.hpp>
#include <subsystems/shooter/Constants.hpp>
#include <string>

namespace Apes668 {
    inline const std::string DEFAULT_ROBOT_SERIAL = "0323CA4B";

    struct RobotConstants final {
        inline static std::string serial = "Unknown";
        inline static std::optional<DrivetrainConstants> drivetrain = std::nullopt;
        inline static std::optional<IntakeConstants> intake = std::nullopt;
        inline static std::optional<ShooterConstants> shooter = std::nullopt;
    };
    
    inline RobotConstants get_robot_constants();

} // namespace Apes668

