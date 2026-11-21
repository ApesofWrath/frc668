package frc.robot;

import org.wpilib.driverstation.XboxController;


public class GameController {
    private XboxController xboxController;

    public GameController(XboxController controller) {
        this.xboxController = controller;
    }

    public void checkButton() {
        System.out.println("A button: " + xboxController.getAButton());
    }

    // @Override
    // public void updateInputs(ControllerIOInputs inputs) {
    //     // inputs.circlePressed = ps4Controller.getCircleButtonPressed();
    //     // inputs.squarePressed = ps4Controller.getSquareButtonPressed();
    //     // inputs.trianglePressed = ps4Controller.getTriangleButtonPressed();
    //     // inputs.crossPressed = ps4Controller.getCrossButtonPressed();
    //     inputs.circlePressed = ps4Controller.getCircleButton();
    //     inputs.squarePressed = ps4Controller.getSquareButton();
    //     inputs.trianglePressed = ps4Controller.getTriangleButton();
    //     inputs.crossPressed = ps4Controller.getCrossButton();

    //     inputs.leftX = ps4Controller.getLeftX();
    //     inputs.leftY = ps4Controller.getLeftY();
    //     inputs.rightX = ps4Controller.getRightX();
    //     inputs.rightY = ps4Controller.getRightY();
    // }

    // private int ccCount = 0;
    // private int stCount = 0;
    // public void seeIfButtonsMakeAMessage(ControllerIOInputs inputs) {
    //     if (inputs.circlePressed && inputs.crossPressed) {
    //         ccCount += 1;
    //         System.out.printf("cir-x %d\n", ccCount);
    //         Logger.recordOutput("cir-x", "⭕️❌");
    //     }
    //     else {
    //         Logger.recordOutput("cir-x", "");
    //     }
    //     if (inputs.squarePressed && inputs.trianglePressed) {
    //         stCount += 1;
    //         System.out.printf("sqr-tri %d\n", stCount);
    //         Logger.recordOutput("sqr-tri", "🔲🔺");
    //     }
    //     else {
    //         Logger.recordOutput("sqr-tri", "");
    //     }

    // }

    // private static final double DEADBAND = 0.1f;
    // public void checkStickDeadbanding(ControllerIOInputs inputs) {
    //     double deadband_leftX = MathUtil.applyDeadband(inputs.leftX, DEADBAND);
    //     Logger.recordOutput("deadband_leftX", deadband_leftX);
    //     double deadband_leftY = MathUtil.applyDeadband(inputs.leftY, DEADBAND);
    //     Logger.recordOutput("deadband_leftY", deadband_leftY);

    //     double deadband_leftX_sm = MathUtil.applyDeadband(inputs.leftX, 0.01f);
    //     Logger.recordOutput("deadband_leftX_sm", deadband_leftX_sm);
    //     double deadband_leftY_sm = MathUtil.applyDeadband(inputs.leftY, 0.01f);
    //     Logger.recordOutput("deadband_leftY_sm", deadband_leftY_sm);

    //     double deadband_rightX = MathUtil.applyDeadband(inputs.leftX, DEADBAND);
    //     Logger.recordOutput("deadband_rightX", deadband_rightX);
    //     double deadband_rightY = MathUtil.applyDeadband(inputs.leftY, DEADBAND);
    //     Logger.recordOutput("deadband_rightY", deadband_rightY);
    // }
}
