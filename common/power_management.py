import math 
import wpilib
import magicbot 
import constants 
from phoenix6 import units 
from subsystem import shooter, drivetrain, intake  
from common import datalog 

class PowerManagement(magicbot.StateMachine):
    turret: shooter.Turret
    hood: shooter.Hood
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    drivetrain: drivetrain.Drivetrain 
    intake: intake.Intake 
    intake_deployer: intake.IntakeDeployer
    shooter_state_machine: shooter.Shooter 
    robot_constants: constants.RobotConstants
    data_logger: datalog.DataLogger

    @magicbot.state(first=True)
    def driving(self, initial_call):
        if initial_call:
            self.turret.setCurrentLimit(40.0)
            self.hood.setCurrentLimit(40.0)
            self.flywheel.setCurrentLimit(40.0)
            self.hopper.setCurrentLimit(40.0)
            self.indexer.setCurrentLimit(40.0)
            self.intake.setCurrentLimit(40.0)
            self.drivetrain.setDriveMotorCurrentLimits(40.0)
            self.drivetrain.setSteerMotorCurrentLimits(40.0)

        if self._currentlyShooting():
            self.next_state_now("driving_and_shooting")
        if self._currentlyIntaking():
            self.next_state_now("driving_and_intaking")

    @magicbot.state
    def driving_and_intaking(self, initial_call): 
        if initial_call:
            self.turret.setCurrentLimit(40.0)
            self.hood.setCurrentLimit(40.0)
            self.flywheel.setCurrentLimit(40.0)
            self.hopper.setCurrentLimit(40.0)
            self.indexer.setCurrentLimit(40.0)
            self.intake.setCurrentLimit(40.0)
            self.drivetrain.setDriveMotorCurrentLimits(40.0)
            self.drivetrain.setSteerMotorCurrentLimits(40.0)

        if self._currentlyShooting():
            self.next_state_now("driving_and_intaking_and_shooting")
        if not self._currentlyDriving():
            self.next_state_now("intaking")
        if not self._currentlyIntaking():
            self.next_state_now("driving")

    @magicbot.state
    def driving_and_shooting(self, initial_call): 
        if initial_call:
            self.turret.setCurrentLimit(40.0)
            self.hood.setCurrentLimit(40.0)
            self.flywheel.setCurrentLimit(40.0)
            self.hopper.setCurrentLimit(40.0)
            self.indexer.setCurrentLimit(40.0)
            self.intake.setCurrentLimit(40.0)
            self.drivetrain.setDriveMotorCurrentLimits(40.0)
            self.drivetrain.setSteerMotorCurrentLimits(40.0)

        if not self._currentlyShooting():
            self.next_state_now("driving")
        if self._currentlyIntaking():
            self.next_state_now("driving_and_intaking_and_shooting")
        if not self._currentlyDriving():
            self.next_state_now("shooting") 

    @magicbot.state
    def intaking(self, initial_call):
        if initial_call:
            self.turret.setCurrentLimit(40.0)
            self.hood.setCurrentLimit(40.0)
            self.flywheel.setCurrentLimit(40.0)
            self.hopper.setCurrentLimit(40.0)
            self.indexer.setCurrentLimit(40.0)
            self.intake.setCurrentLimit(40.0)
            self.drivetrain.setDriveMotorCurrentLimits(40.0)
            self.drivetrain.setSteerMotorCurrentLimits(40.0)

        if self._currentlyShooting():
            self.next_state_now("intaking_and_shooting")
        if self._currentlyDriving():
            self.next_state_now("driving_and_intaking")

    @magicbot.state
    def intaking_and_shooting(self, initial_call):
        if initial_call:
            self.turret.setCurrentLimit(40.0)
            self.hood.setCurrentLimit(40.0)
            self.flywheel.setCurrentLimit(40.0)
            self.hopper.setCurrentLimit(40.0)
            self.indexer.setCurrentLimit(40.0)
            self.intake.setCurrentLimit(40.0)
            self.drivetrain.setDriveMotorCurrentLimits(40.0)
            self.drivetrain.setSteerMotorCurrentLimits(40.0)

        if self._currentlyDriving():
            self.next_state_now("driving_and_intaking_and_shooting")
        if not self._currentlyShooting():
            self.next_state_now("intaking")
        if not self._currentlyIntaking():
            self.next_state_now("shooting") 

    @magicbot.state
    def shooting(self, initial_call):
        if initial_call:
            self.turret.setCurrentLimit(40.0)
            self.hood.setCurrentLimit(40.0)
            self.flywheel.setCurrentLimit(40.0)
            self.hopper.setCurrentLimit(40.0)
            self.indexer.setCurrentLimit(40.0)
            self.intake.setCurrentLimit(40.0)
            self.drivetrain.setDriveMotorCurrentLimits(40.0)
            self.drivetrain.setSteerMotorCurrentLimits(40.0)

        if self._currentlyDriving():
            self.next_state_now("driving_and_shooting")
        if self._currentlyIntaking():
            self.next_state_now("intaking_and_shooting")

    @magicbot.state
    def driving_and_intaking_and_shooting(self, initial_call):
        if initial_call:
            self.turret.setCurrentLimit(40.0)
            self.hood.setCurrentLimit(40.0)
            self.flywheel.setCurrentLimit(40.0)
            self.hopper.setCurrentLimit(40.0)
            self.indexer.setCurrentLimit(40.0)
            self.intake.setCurrentLimit(40.0)
            self.drivetrain.setDriveMotorCurrentLimits(40.0)
            self.drivetrain.setSteerMotorCurrentLimits(40.0)

        if not self._currentlyShooting():
            self.next_state_now("driving_and_intaking")
        if not self._currentlyDriving():
            self.next_state_now("intaking_and_shooting")
        if not self._currentlyIntaking():
            self.next_state_now("driving_and_shooting") 

    def computeTotalCurrentDrawn(self) -> units.ampere:
        intake_currents = float(self.intake.rollerSupplyCurrent()) + float(self.intake_deployer.deploySupplyCurrent()) 
        drive_currents = float(self.drivetrain.backLeftDriveSupplyCurrent()) + float(self.drivetrain.frontLeftDriveSupplyCurrent()) + float(self.drivetrain.backRightDriveSupplyCurrent()) + float(self.drivetrain.frontRightDriveSupplyCurrent())
        steer_currents = float(self.drivetrain.backLeftSteerSupplyCurrent()) + float(self.drivetrain.frontLeftSteerSupplyCurrent()) + float(self.drivetrain.backRightSteerSupplyCurrent()) + float(self.drivetrain.frontRightSteerSupplyCurrent())
        shooter_currents = float(self.turret.supplyCurrent()) + float(self.hood.supplyCurrent()) + float(self.flywheel.supplyCurrent()) + float(self.indexer.frontSupplyCurrent()) + float(self.indexer.backSupplyCurrent()) + float(self.hopper.leftSupplyCurrent()) + float(self.hopper.rightSupplyCurrent())
        
        total_current = intake_currents + drive_currents + steer_currents + shooter_currents
        return total_current
    
    def _currentlyDriving(self) -> bool:
        return True if self.drivetrain._robotIsMoving() or self.drivetrain.getBrakeEnabled() else False
    
    def _currentlyIntaking(self) -> bool:
        return self.intake.getActive()
    
    def _currentlyShooting(self) -> bool:
        return self.shooter_state_machine.getDriverWantsFeed()
    
    def _logData(self) -> None:
        self.data_logger.logDouble("Total Power Drawn", float(self.computeTotalCurrentDrawn()))