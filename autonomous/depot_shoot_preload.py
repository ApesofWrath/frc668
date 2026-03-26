from autonomous import auto_base


class DepotShootPreload(auto_base.AutoBase):
    """Start on the depot side of the bumps, shoot pre-loaded fuel."""

    MODE_NAME = "Depot Shoot Preload"
    TRAJECTORY_NAME = "depot_shoot_preload"
