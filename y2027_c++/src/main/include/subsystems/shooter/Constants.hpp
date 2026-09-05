#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <string>

using namespace ctre::phoenix6;

namespace Apes668 {
    struct HopperConstants final {
        int left_motor_can_id = 0;
        std::string left_motor_can_bus = "";
        signals::InvertedValue left_motor_inverted = signals::InvertedValue::CounterClockwise_Positive;

        // ...
    };

    struct ShooterConstants final {
        HopperConstants hopper = HopperConstants();

        // ...
    };
}