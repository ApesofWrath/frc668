from autonomous import auto_base


class NeutralZoneShootStartingDepotSide(auto_base.AutoBase):
    """
    1. Start on the depot-side bump.
    2. Drive over bump into the neutral zone, do 2 intake sweeps.
    3. Drive over bump back into alliance zone.
    4. Shoot fuel.
    """

    MODE_NAME = "Neutral Zone Shoot Starting Depot Side"
    TRAJECTORY_NAME = "neutral_zone_shoot_starting_depot_side"
    SHOOT_DURATION_SECONDS = 8.0
