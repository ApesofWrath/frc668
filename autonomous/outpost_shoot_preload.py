from autonomous import auto_base


class OutpostShootPreload(auto_base.AutoBase):
    """Start at the outpost-side bump, shoot pre-loaded fuel."""

    MODE_NAME = "Outpost Shoot Preload"
    TRAJECTORY_NAME = "outpost_shoot_preload"
    SHOOT_DURATION_SECONDS = 8.0
