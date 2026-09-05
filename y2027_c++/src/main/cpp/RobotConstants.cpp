#include <RobotConstants.hpp>
#include <frc/RobotBase.h>
#include <frc/Errors.h>
#include <string>

using namespace frc;

namespace Apes668 {
    RobotConstants get_robot_constants() {
        RobotConstants robot_constants = RobotConstants();

        if (RobotBase::IsSimulation()) {
            robot_constants.serial = DEFAULT_ROBOT_SERIAL;
            FRC_ReportWarning("Running in simulation - using default robot constants");
        }
        else {
            // I don't understand why the env is queried for the robot's serial number
            robot_constants.serial = "uh, what's happening here?";
        }

        robot_constants.drivetrain = DRIVETRAIN_CONSTANTS[robot_constants.serial];

        return robot_constants;
    }
}