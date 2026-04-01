import math 
import wpilib
import magicbot 
import constants 
from subsystem import shooter, drivetrain, intake  
import joystick

class PowerManagement(magicbot.StateMachine):
    turret: shooter.Turret
    hood: shooter.Hood
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    drivetrain: drivetrain.Drivetrain 
    intake: intake.Intake 
    joystick: joystick.DriverController
    robot_constants: constants.RobotConstants

    def setup(self):
        self.driver_controller = joystick.DriverController(
                wpilib.XboxController(0),
                self.robot_constants.drivetrain.drive_options,
            )

    def powerDistribution(self):
        self.engage()

    @magicbot.state(first=True)
    def idling(self):
        if self._currentlyDriving() and not self._currentlyIntaking() and not self._currentlyShooting() and not self._currentlyShootingFromPreset():
            self.next_state("driving")
        elif self._currentlyDriving() and self._currentlyIntaking() and not self._currentlyShooting() and not self._currentlyShootingFromPreset():
            self.next_state("driving_and_intaking")
        elif self._currentlyShootingFromPreset() or (self._currentlyShooting() and not self._currentlyDriving()):
            self.next_state("shooting_while_stationary")
        elif self._currentlyDriving() and self._currentlyShooting():
            self.next_state("shooting_on_the_move")
        else:
            # Set default current limits for everything 
            pass        

    @magicbot.state
    def driving(self):
        # Drivetrain current limits increased, all other mechanisms decreased
        pass 

    @magicbot.state
    def driving_and_intaking(self):
        # Drivetrain and intake current limits increased, shooter-related decreased
        pass

    @magicbot.state
    def shooting_while_stationary(self):
        # Shooter-related current limits increased, drivetrain and intaking decreased
        pass

    @magicbot.state
    def shooting_on_the_move(self):
        # Shooter-related current limits increased, drivetrain somewhat limited, intaking low limit
        pass 

    def computeTotalPowerDrawn(self):
        # Calculate the total current drawn from all motors
        pass 
    
    
    def _currentlyDriving(self) -> bool:
        if self.driver_controller.getDriveCommand() is not None:
            return True 
        else: 
            return False 
    
    def _currentlyIntaking(self) -> bool:
        return self.driver_controller.toggleIntake()
    
    def _currentlyShooting(self) -> bool:
        return self.driver_controller.feedFuel() 
    
    def _currentlyShootingFromPreset(self) -> bool:
        return True if self.driver_controller.shootFromBehindTower() or self.driver_controller.shootFromLeftTrench() or self.driver_controller.shootFromRightTrench() else False 