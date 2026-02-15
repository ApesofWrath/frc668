import magicbot
import phoenix6
import wpilib

import robot
from subsystem import shooter


class TunerBot(robot.MyRobot):
    """Robot class for testing and tuning various mechanisms.

    This class extends the base robot by adding components that help us tune
    various mechanisms, and provides more diagnostic information.
    """

    flywheel_tuner: shooter.FlywheelTuner
    hood_tuner: shooter.hood.HoodTuner

    #to run, type in the terminal: py -3 -m robotpy --main tunerbot.py deploy 