import numpy as np


class DeviceBase:
    def __init__(self):
        self._delta_rot = np.zeros(3)
        self._delta_pos = np.zeros(3)
        self._gripper_value = 0.0
