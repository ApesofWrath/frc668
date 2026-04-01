from autonomous import auto_base


class NeutralZoneDepotShootStartingDepotSideNoNZParking(auto_base.AutoBase):
    """
    1. Start on the depot-side bump.
    2. Drive over bump into the neutral zone, do 2 intake sweeps.
    3. Drive over bump back into alliance zone, go to depot. Begin shooting.
    4. Do 1 intake sweep over depot (outward).
    5. Drive towards hub, allowing for time to finish shooting fuel.
    """

    MODE_NAME = (
        "Neutral Zone and Depot Intake Shoot Starting Depot Side, No NZ Parking"
    )
    TRAJECTORY_NAME = (
        "neutral_zone_depot_shoot_starting_depot_side_no_nz_parking"
    )
    SHOOT_ON_THE_MOVE = True
    SHOOT_DURATION_SECONDS = 5.0
