from magicbot import state, StateMachine
from subsystem import shooter

class ShooterStateMachine(StateMachine):

    is_shooting = False

    turret: shooter.Turret
    hood: shooter.Hood
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    hub_tracker: shooter.HubTracker
    
    @state(first=True)
    def targeting(self,initial_call):
        """Attempting to get to target postion"""
        if self._get_shooter_state_is_target_state_default() and self.is_shooting:
            self.next_state_now("shooting")
            
        if initial_call or self.hopper.get_enabled() or self.indexer.get_enabled():
            self.hopper.setEnabled(False)
            self.indexer.setEnabled(False)
        self.logger.info(self._get_shooter_state_is_target_state_default())

    @state
    def shooting(self,initial_call):
        """Actively shooting"""
        if not self.is_shooting:
            self.next_state("targeting")
        if not self._get_shooter_state_is_target_state_default():
            self.next_state("targeting")
            return
        
        if initial_call or not self.hopper.get_enabled() or not self.indexer.get_enabled():
            self.hopper.setEnabled(True)
            self.indexer.setEnabled(True)

    def _get_shooter_state_is_target_state_default(self) -> bool:
        return self._get_shooter_state_is_target_state(7,3,1)

    def _get_shooter_state_is_target_state(self,turret_tolerance_degrees: float,
                                           hood_tolerance_degrees: float,
                                           flywheel_tolerance_rotations_per_second: float) -> bool:
        turret_error = abs(self.hub_tracker.get_turret_target_angle_degrees() - self.turret.get_turret_angle())
        hood_error = abs(self.hood.get_target_position_deg() - self.hood.get_measured_angle_degrees()) # TODO: use hubtracker
        flywheel_error = abs(self.flywheel.get_target_rps() - self.flywheel.flywheel_encoder.get_velocity().value)
        # TODO: use hubtracker
        return ( (turret_error <= turret_tolerance_degrees)
               & (hood_error <= hood_tolerance_degrees)
               & (flywheel_error <= flywheel_tolerance_rotations_per_second) )