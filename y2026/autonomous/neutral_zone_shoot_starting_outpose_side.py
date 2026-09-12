from autonomous import auto_base


class NeutralZoneShootStartingOutpostSide(auto_base.AutoBase):
    """
    1. Start on the outpost-side bump.
    2. Drive over bump into the neutral zone, do 2 intake sweeps.
    3. Drive bacl over same bump into our alliance zone.
    4. Shoot fuel.
    """

    MODE_NAME = "Neutral Zone Shoot Starting Outpost Side"
    TRAJECTORY_NAME = "neutral_zone_shoot_starting_outpost_side"
    SHOOT_DURATION_SECONDS = 8.0
