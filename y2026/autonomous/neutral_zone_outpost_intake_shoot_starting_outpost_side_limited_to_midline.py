from autonomous import auto_base


class NeutralZoneOutpostIntakeShootStartingOutpostSideLimitedToMidline(
    auto_base.AutoBase
):
    """
    1. Start on the outpost-side bump.
    2. Drive over bump into the neutral zone, do 2 intake sweeps.
    3. Drive bacl over same bump into our alliance zone.
    4. Shoot fuel.
    """

    MODE_NAME = "Neutral Zone and Outpost Intake Shoot Starting Outpost Side, Limited to Half Field"
    TRAJECTORY_NAME = "neutral_zone_outpost_intake_shoot_starting_outpost_side_limited_to_midline"
    SHOOT_DURATION_SECONDS = 10.0
