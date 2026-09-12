from autonomous import auto_base


class NeutralZoneDepotShootStartingDepotSide(auto_base.AutoBase):
    """
    1. Start on the depot-side bump.
    2. Drive over bump into the neutral zone, do 2 intake sweeps.
    3. Drive over bump back into alliance zone, go to depot. Begin shooting.
    4. Do 1 intake sweep over depot (outward).
    5. Drive towards hub, allowing for time to finish shooting fuel.
    6. Drive over bump back into the neutral zone.
    """

    MODE_NAME = "Neutral Zone and Depot Intake Shoot Starting Depot Side"
    TRAJECTORY_NAME = "neutral_zone_depot_shoot_starting_depot_side"
    SHOOT_ON_THE_MOVE = True
