#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/swerve/SwerveModule.hpp>
#include <RobotConstants.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>

using namespace ctre::phoenix6;

namespace Apes668 {
    class Drivetrain : public frc2::SubsystemBase {
    public:
        Drivetrain();

        void setup() {
            RobotConstants robot_constants = get_robot_constants();
            if (robot_constants.drivetrain.has_value()) {
                const DrivetrainConstants& constants = *robot_constants.drivetrain;

                auto constants_factory = swerve::SwerveModuleConstantsFactory<
                    configs::TalonFXConfiguration,
                    configs::TalonFXConfiguration,
                    configs::CANcoderConfiguration>();
                constants_factory
                    .WithDriveMotorGearRatio(constants.common.drive_motor_gear_ratio)
                    .WithSteerMotorGearRatio(constants.common.steer_motor_gear_ratio)
                    .WithCouplingGearRatio(constants.common.coupling_gear_ration)
                    .WithWheelRadius(constants.common.wheel_radius)
                    .WithSteerMotorGains(constants.common.steer_motor_gains)
                    .WithDriveMotorGains(constants.common.drive_motor_gains)
                    .WithSteerMotorClosedLoopOutput(constants.common.steer_motor_closed_loop_output)
                    .WithDriveMotorClosedLoopOutput(constants.common.drive_motor_closed_loop_output)
                    .WithSlipCurrent(constants.common.slip_current)
                    .WithSpeedAt12Volts(constants.common.speed_at12_volts)
                    .WithDriveMotorType(constants.common.drive_motor_type)
                    .WithSteerMotorType(constants.common.steer_motor_type)
                    .WithFeedbackSource(constants.common.feedback_source)
                    .WithDriveMotorInitialConfigs(constants.common.drive_motor_initial_configs)
                    .WithSteerMotorInitialConfigs(constants.common.steer_motor_initial_configs)
                    .WithEncoderInitialConfigs(constants.common.encoder_initial_configs);

                
                swerve_drive.emplace(
                    swerve::SwerveDrivetrainConstants()
                        .WithCANBusName(constants.drivetrain.can_bus_name)
                        .WithPigeon2Id(constants.drivetrain.pigeon2_id)
                        .WithPigeon2Configs(constants.drivetrain.pigeon2_configs),

                    constants_factory.CreateModuleConstants(
                        constants.front_left.steer_motor_id,
                        constants.front_left.drive_motor_id,
                        constants.front_left.encoder_id,
                        constants.front_left.encoder_offset,
                        constants.front_left.location_x,
                        constants.front_left.location_y,
                        constants.front_left.drive_motor_inverted,
                        constants.front_left.steer_motor_inverted,
                        constants.front_left.encoder_inverted
                    ),

                    constants_factory.CreateModuleConstants(
                        constants.front_right.steer_motor_id,
                        constants.front_right.drive_motor_id,
                        constants.front_right.encoder_id,
                        constants.front_right.encoder_offset,
                        constants.front_right.location_x,
                        constants.front_right.location_y,
                        constants.front_right.drive_motor_inverted,
                        constants.front_right.steer_motor_inverted,
                        constants.front_right.encoder_inverted
                    ),

                    constants_factory.CreateModuleConstants(
                        constants.back_left.steer_motor_id,
                        constants.back_left.drive_motor_id,
                        constants.back_left.encoder_id,
                        constants.back_left.encoder_offset,
                        constants.back_left.location_x,
                        constants.back_left.location_y,
                        constants.back_left.drive_motor_inverted,
                        constants.back_left.steer_motor_inverted,
                        constants.back_left.encoder_inverted
                    ),

                    constants_factory.CreateModuleConstants(
                        constants.back_right.steer_motor_id,
                        constants.back_right.drive_motor_id,
                        constants.back_right.encoder_id,
                        constants.back_right.encoder_offset,
                        constants.back_right.location_x,
                        constants.back_right.location_y,
                        constants.back_right.drive_motor_inverted,
                        constants.back_right.steer_motor_inverted,
                        constants.back_right.encoder_inverted
                    )
                );

                _x_controller = frc::PIDController(constants.trajectory_following.x_kp, 0.0f, 0.0f);
                _y_controller = frc::PIDController(constants.trajectory_following.y_kp, 0.0f, 0.0f);
                _heading_controller = frc::PIDController(constants.trajectory_following.heading_kp, 0.0f, 0.0f);
                _heading_controller.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
                auto x = swerve::requests::FieldCentric().WithDriveRequestType(swerve::DriveRequestType::Velocity);
            }
        }

        void execute() {

        }

        void Periodic() override {

        }

    private:
        std::optional<swerve::SwerveDrivetrain<hardware::TalonFX, hardware::TalonFX, hardware::CANcoder>> swerve_drive;
        frc::PIDController _x_controller;
        frc::PIDController _y_controller;
        frc::PIDController _heading_controller;
        
    };
}
