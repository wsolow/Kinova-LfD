from typing import Any

from kinova_lfd.envs.gen3_kortex import Gen3KortexEnv, Gen3KortexEnvCfg
from kinova_lfd.envs.kortex import KortexEnv
from kinova_lfd.envs.kortex.kortex_bridge import setup_kortex_tcp_udp


def create_kortex_env(task_name: str, cfg: dict[str, Any]) -> KortexEnv:
    """Create a Kortex environment for the given task name and configuration."""
    tcp_udp = setup_kortex_tcp_udp(ip=cfg["robot_ip"])
    if task_name == "Kortex-Gen3-v0":
        env_cfg = Gen3KortexEnvCfg(**cfg)
        return Gen3KortexEnv(cfg=env_cfg, kortex_tcp_udp=tcp_udp)

    raise ValueError(f"Invalid task name: {task_name}")
