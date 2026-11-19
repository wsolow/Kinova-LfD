"""Keyboard controller for SE(2) control (standalone, no Omni/Carb dependency)."""

from __future__ import annotations

import sys
import termios
from collections.abc import Callable

import numpy as np
import rclpy
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import PoseStamped, Twist
from kinova_lfd_ros.core.math import base_to_tcp_twist, euler_xyz_to_rotvec
from kinova_lfd_ros.teleop.devices.device_base import DeviceBase
from pynput import keyboard as pynput_keyboard
from rclpy.action import ActionClient
from rclpy.node import Node


class Se3Keyboard(DeviceBase):
    """A keyboard controller for sending SE(3) commands as delta poses and binary command (open/close).

    Uses pynput instead of Omniverse's carb/omni keyboard interface, so it can run
    outside of an Isaac Sim / Omniverse context.

    Key bindings:
        ============================== ================= =================
        Description                    Key (+ve axis)    Key (-ve axis)
        ============================== ================= =================
        Toggle gripper (open/close)    R
        Move along x-axis              W                 S
        Move along y-axis              A                 D
        Move along z-axis              Q                 E
        Rotate along x-axis            Z                 X
        Rotate along y-axis            T                 G
        Rotate along z-axis            C                 V
        ============================== ================= =================
    """

    def __init__(self):
        super().__init__()

        # Command buffers
        self._close_gripper = False
        self._delta_pos = np.zeros(3)
        self._delta_rot = np.zeros(3)
        self._home_flag = False

        # terminal settings
        self._fd = sys.stdin.fileno()
        self._old_term_settings = termios.tcgetattr(self._fd)

        self._key_listener = pynput_keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._key_listener.start()

        self._additional_callbacks: dict[str, Callable] = {}
        self._create_key_bindings()
        self._pressed_keys: set[str] = set()

    def _key_to_char(self, key) -> str | None:
        """Convert a pynput key object to an uppercase character string, or None."""
        try:
            # Regular character keys
            return key.char.upper()
        except AttributeError:
            # Special keys (shift, ctrl, etc.) — not used here
            return None

    def _on_press(self, key):
        char = self._key_to_char(key)
        if char is None:
            return

        if char == "F":
            self.reset()
            return

        if char == "R":
            self._close_gripper = not self._close_gripper
            self._gripper_value = 1.0 if self._close_gripper else 0.0
            return
        if char == "P":
            self._home_flag = True

        # Guard against key-repeat events firing duplicate additions
        if char in self._pressed_keys:
            return
        self._pressed_keys.add(char)

        if char in self._POS_KEYS:
            self._delta_pos += self._INPUT_KEY_MAPPING[char]
        elif char in self._ROT_KEYS:
            self._delta_rot += self._INPUT_KEY_MAPPING[char]

        # User-registered callbacks
        if char in self._additional_callbacks:
            self._additional_callbacks[char]()

    def _on_release(self, key) -> None:
        char = self._key_to_char(key)
        if char is None:
            return

        if char == "P":
            self._home_flag = False

        self._pressed_keys.discard(char)

        if char in self._POS_KEYS:
            self._delta_pos -= self._INPUT_KEY_MAPPING[char]
        elif char in self._ROT_KEYS:
            self._delta_rot -= self._INPUT_KEY_MAPPING[char]

    def _create_key_bindings(self) -> None:
        self._POS_KEYS = {"W", "S", "A", "D", "Q", "E"}
        self._ROT_KEYS = {"Z", "X", "T", "G", "C", "V"}

        self._INPUT_KEY_MAPPING = {
            # x-axis (forward/back)
            "W": np.asarray([1.0, 0.0, 0.0]),
            "S": np.asarray([-1.0, 0.0, 0.0]),
            # y-axis (left/right)
            "A": np.asarray([0.0, 1.0, 0.0]),
            "D": np.asarray([0.0, -1.0, 0.0]),
            # z-axis (up/down)
            "Q": np.asarray([0.0, 0.0, 1.0]),
            "E": np.asarray([0.0, 0.0, -1.0]),
            # roll (around x)
            "Z": np.asarray([1.0, 0.0, 0.0]),
            "X": np.asarray([-1.0, 0.0, 0.0]),
            # pitch (around y)
            "T": np.asarray([0.0, 1.0, 0.0]),
            "G": np.asarray([0.0, -1.0, 0.0]),
            # yaw (around z)
            "C": np.asarray([0.0, 0.0, 1.0]),
            "V": np.asarray([0.0, 0.0, -1.0]),
        }

    def __str__(self) -> str:
        msg = f"Keyboard Controller for SE(3): {self.__class__.__name__}\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tToggle gripper (open/close): R\n"
        msg += "\tMove arm along x-axis: W/S\n"
        msg += "\tMove arm along y-axis: A/D\n"
        msg += "\tMove arm along z-axis: Q/E\n"
        msg += "\tRotate arm along x-axis: Z/X\n"
        msg += "\tRotate arm along y-axis: T/G\n"
        msg += "\tRotate arm along z-axis: C/V"
        return msg
