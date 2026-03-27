from autonomous import auto_base


class DepotIntakeShoot(auto_base.AutoBase):
    """Start from the depot side of the bumps, intake fuel from the depot, shoot fuel."""

    MODE_NAME = "Depot Intake Shoot"
    TRAJECTORY_NAME = "depot_intake_shoot"
