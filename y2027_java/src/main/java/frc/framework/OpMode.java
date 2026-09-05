package frc.framework;

/**
 * The current state of the robot
 */
public enum OpMode {
    /**
     * The robot is disabled, e.g, before a match, or in the transition between autonomous and teleop
     */
    Disabled,
    /**
     * The game is in the initial autonomous period
     */
    Autonomous,
    /**
     * The game is in the driver controlled period
     */
    Teleop,
    /**
     * The robot is running in a testing environment
     */
    Test;
}
