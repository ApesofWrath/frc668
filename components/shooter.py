

class Shooter:
    
    def __init__(self):
        # define all shooter moters and encoders and stuff
    
    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the shooter based off current state.
        """
        # spin the moters and stuff
        ''' (pseudocode)
        self.turretmotor.setAngle(self.turretAngle)
        self.hoodmotor.setAngle(self.hoodAngle)
        self.flywheelmotor.setVelocity(self.flywheelTargetVelocity) # needs to get the PID tuning i think

        do any SmartDashboard stuff, if needed
        '''

    def isManual(self):
        """
        Returns whether the shooter is being controlled by the operator
        
        Currently just returns True, but will default to False when auto align is added,
        and could also return False if the autoalign doesnt work
        """
        return True