#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <string>

using namespace ctre::phoenix6;

namespace Apes668 {
    struct IntakeConstants final {
        int roller_top_motor_can_id = 0;
        std::string roller_top_motor_can_bus = "";
        signals::InvertedValue roller_top_motor_inverted = signals::InvertedValue::CounterClockwise_Positive;

        // ...
    };
}