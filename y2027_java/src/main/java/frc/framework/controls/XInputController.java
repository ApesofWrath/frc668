package frc.framework.controls;

import edu.wpi.first.wpilibj.XboxController;

public class XInputController  implements Controller {
    private XboxController controller;

    public XInputController(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public boolean isButtonPressed(Button button) {
        return switch (button) {
            case A -> controller.getAButton();
            case B -> controller.getBButton();
            case L1 -> controller.getLeftBumperButton();
            case L3 -> controller.getLeftStickButton();
            case R1 -> controller.getRightBumperButton();
            case R3 -> controller.getRightStickButton();
            case X -> controller.getXButton();
            case Y -> controller.getYButton();
            default -> false;
        };
    }

    @Override
    public double getAxis(Axis axis) {
        return switch (axis) {
            case LEFT_HORIZONTAL -> controller.getLeftX();
            case LEFT_VERTICAL -> controller.getLeftY();
            case RIGHT_HORIZONTAL -> controller.getRightX();
            case RIGHT_VERTICAL -> controller.getRightY();
            default -> 0;
        };
    }
}
