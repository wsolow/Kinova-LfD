"""kortex_env_cfg.py.

Main configuration for Kortex Env CFG file

Written by Will Solow, 2026

"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class KortexEnvCfg:
    """The configuration class for Kortex Envs."""

    robot_ip: str = "www.xxx.yyy.zzz"
    """IP of the robot"""

    use_tabletop_camera: bool = False

    use_wrist_camera: bool = True

    dt: float = 1.0 / 30.0

    decimation: float = 1.0

    use_udp: bool = False
    """If to use UDP for arm commands and feedback."""

    urdf_fpath: str = "kinova_lfd/envs/assets/kinova_gen3/gen3_2f85.urdf"

    ee_frame: str = "tool_frame"
