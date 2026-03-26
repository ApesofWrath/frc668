from autonomous import auto_base


class NeutralZoneDepotShootStartingDepotSide(auto_base.AutoBase):
    """Start from the depot side of the bumps, intake fuel from neutral zone, shoot fuel."""

    MODE_NAME = "Neutral Zone and Depot Intake Shoot Starting Depot Side"
    TRAJECTORY_NAME = "neutral_zone_depot_shoot_starting_depot_side"
