from autonomous import auto_base


class DepotIntakeShoot(auto_base.AutoBase):
    """
    1. Start at the depot-side bump.
    2. Drive to the depot, do 1 intake sweep.
    3. Drive back near the hub, and shoot fuel.
    """

    MODE_NAME = "Depot Intake Shoot"
    TRAJECTORY_NAME = "depot_intake_shoot"
    SHOOT_DURATION_SECONDS = 8.0
