#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/core/CorePigeon2.hpp>
#include <units/current.h>
#include <units/angle.h>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ctre::phoenix6;

namespace Apes668 {
    struct SwerveModuleCommonConstants final {
        float drive_motor_gear_ratio = 4.67;
        float steer_motor_gear_ratio = 25.9;
        float coupling_gear_ration = 0.0;
        units::meter_t wheel_radius = 0.04445_m;

        configs::Slot0Configs steer_motor_gains = configs::Slot0Configs();
        configs::Slot0Configs drive_motor_gains = configs::Slot0Configs();

        swerve::ClosedLoopOutputType steer_motor_closed_loop_output = swerve::ClosedLoopOutputType::Voltage;
        swerve::ClosedLoopOutputType drive_motor_closed_loop_output = swerve::ClosedLoopOutputType::Voltage;

        units::ampere_t slip_current = 120.0_A;
        units::meters_per_second_t speed_at12_volts = 12.0_mps;

        swerve::DriveMotorArrangement drive_motor_type = swerve::DriveMotorArrangement::TalonFX_Integrated;
        swerve::SteerMotorArrangement steer_motor_type = swerve::SteerMotorArrangement::TalonFX_Integrated;

        swerve::SteerFeedbackType feedback_source = swerve::SteerFeedbackType::FusedCANcoder;

        configs::TalonFXConfiguration drive_motor_initial_configs = 
            configs::TalonFXConfiguration{}
                .WithCurrentLimits(
                    configs::CurrentLimitsConfigs{}
                        .WithSupplyCurrentLimit(35_A)
                        .WithSupplyCurrentLimitEnable(true));


        configs::TalonFXConfiguration steer_motor_initial_configs = 
            configs::TalonFXConfiguration{}
                .WithCurrentLimits(
                    configs::CurrentLimitsConfigs{}
                        .WithSupplyCurrentLimit(30_A)
                        .WithSupplyCurrentLimitEnable(true));

        configs::CANcoderConfiguration encoder_initial_configs = configs::CANcoderConfiguration();

        auto operator<=>(const SwerveModuleCommonConstants&) const = delete;
    };

    struct SwerveModuleConstants final {
        int steer_motor_id = 0;
        int drive_motor_id = 0;
        int encoder_id = 0;
        units::turn_t encoder_offset = 0_tr;
        units::meter_t location_x = 0_m;
        units::meter_t location_y = 0_m;
        bool drive_motor_inverted = false;
        bool steer_motor_inverted = false;
        bool encoder_inverted = false;

        auto operator<=>(const SwerveModuleConstants&) const = delete;
    };

    struct SwerveDrivetrainConstants final {
        std::string can_bus_name = ""; 
        int pigeon2_id = 0;
        std::optional<configs::Pigeon2Configuration> pigeon2_configs = std::nullopt;

        auto operator<=>(const SwerveDrivetrainConstants&) const = delete;
    };

    struct DriveOptions final {
        float max_linear_speed_meters_per_second = 6.0;
        float max_linear_acceleration_meters_per_second_squared = 3.0;
        float max_angular_speed_radians_per_second = 6.0;
        float max_angular_acceleration_radians_per_second_squared = 0.5;

        auto operator<=>(const DriveOptions&) const = delete;
    };

    struct VisionConstants final {
        std::vector<std::string> limelights;

        auto operator<=>(const VisionConstants&) const = delete;
    };

    struct TrajectoryFollowingConstants final {
        float x_kp = 0.0;
        float x_ki = 0.0;
        float x_kd = 0.0;
        float y_kp = 0.0;
        float y_ki = 0.0;
        float y_kd = 0.0;
        float heading_kp = 0.0;
        float heading_ki = 0.0;
        float heading_kd = 0.0;

        auto operator<=>(const TrajectoryFollowingConstants&) const = delete;
    };

    struct DrivetrainConstants final {
        SwerveModuleCommonConstants common = SwerveModuleCommonConstants();
        SwerveModuleConstants front_left = SwerveModuleConstants();
        SwerveModuleConstants front_right = SwerveModuleConstants();
        SwerveModuleConstants back_left = SwerveModuleConstants();
        SwerveModuleConstants back_right = SwerveModuleConstants();
        SwerveDrivetrainConstants drivetrain = SwerveDrivetrainConstants();
        DriveOptions drive_options = DriveOptions();
        VisionConstants vision = VisionConstants();
        TrajectoryFollowingConstants trajectory_following = TrajectoryFollowingConstants();
    };

    inline std::unordered_map<std::string, DrivetrainConstants> DRIVETRAIN_CONSTANTS = {
        // Test chassis
        { "0323800E", DrivetrainConstants() },
        // Alphabot
        { "023AC96C", DrivetrainConstants() },
        // Juno
        { "0323CA4B", [] {
            DrivetrainConstants constants;

            constants.common.steer_motor_gains.kS = 0.1321525;
            // ...
            constants.common.drive_motor_gains.kS = 0.309905;
            // ...

            return constants;
            }()
        }
    };
}
