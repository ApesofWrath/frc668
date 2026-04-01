from autonomous import auto_base


class DepotShootPreload(auto_base.AutoBase):
    """Start at the depot-side bump, shoot pre-loaded fuel."""

    MODE_NAME = "Depot Shoot Preload"
    TRAJECTORY_NAME = "depot_shoot_preload"
    SHOOT_DURATION_SECONDS = 8.0
