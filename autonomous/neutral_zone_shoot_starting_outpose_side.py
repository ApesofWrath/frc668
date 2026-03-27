from autonomous import auto_base


class NeutralZoneShootStartingOutpostSide(auto_base.AutoBase):
    """Start from the outpost side of the bumps, intake fuel from neutral zone, shoot fuel."""

    MODE_NAME = "Neutral Zone Shoot Starting Outpost Side"
    TRAJECTORY_NAME = "neutral_zone_shoot_starting_outpost_side"
