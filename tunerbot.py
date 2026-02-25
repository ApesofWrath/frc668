import magicbot
import phoenix6
import wpilib

import robot
from subsystem import shooter


class TunerBot(robot.MyRobot):
    """Robot class for testing and tuning various mechanisms.

    This class extends the base robot by adding components that help us tune
    various mechanisms, and provides more diagnostic information.

    Deploy the TunerBot program to the RoboRIO with the command:
    ```
    py -3 -m robotpy --main tunerbot.py deploy
    ```
    """

    flywheel_tuner: shooter.FlywheelTuner
    hood_tuner: shooter.HoodTuner
    hopper_tuner: shooter.HopperTuner
    indexer_tuner: shooter.IndexerTuner
    turret_tuner: shooter.TurretTuner

    def createObjects(self) -> None:
        super().createObjects()
        self._tuning_mode = True
