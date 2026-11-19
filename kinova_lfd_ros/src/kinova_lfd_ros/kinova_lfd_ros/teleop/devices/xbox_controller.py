from __future__ import annotations

import select
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Optional

import numpy as np
from evdev import InputDevice, ecodes, list_devices
from kinova_lfd_ros.teleop.devices.device_base import DeviceBase

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator


class DPadDirection(Enum):
    """State of pressable pad."""

    CENTER = "center"
    UP = "up"
    DOWN = "down"
    LEFT = "left"
    RIGHT = "right"


@dataclass
class StickState:
    """State of joystick."""

    x: float = 0.0  # -1.0 (left/down) to 1.0 (right/up)
    y: float = 0.0
    pressed: bool = False


@dataclass
class XboxControllerState:
    """Snapshot of all readable inputs."""

    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False
    lb: bool = False  # left bumper
    rb: bool = False  # right bumper
    back: bool = False  # View / Select
    start: bool = False  # Menu / Start
    guide: bool = False  # Xbox button
    left_stick: StickState = field(default_factory=StickState)
    right_stick: StickState = field(default_factory=StickState)
    lt: float = 0.0  # left trigger, 0.0-1.0
    rt: float = 0.0  # right trigger, 0.0-1.0
    dpad: DPadDirection = DPadDirection.CENTER

    def as_dict(self) -> dict:
        """Return xbox state as dictionary."""
        return {
            "a": self.a,
            "b": self.b,
            "x": self.x,
            "y": self.y,
            "lb": self.lb,
            "rb": self.rb,
            "back": self.back,
            "start": self.start,
            "guide": self.guide,
            "left_stick": {
                "x": self.left_stick.x,
                "y": self.left_stick.y,
                "pressed": self.left_stick.pressed,
            },
            "right_stick": {
                "x": self.right_stick.x,
                "y": self.right_stick.y,
                "pressed": self.right_stick.pressed,
            },
            "lt": self.lt,
            "rt": self.rt,
            "dpad": self.dpad.value,
        }


_BUTTON_MAP = {
    ecodes.BTN_SOUTH: "a",
    ecodes.BTN_EAST: "b",
    ecodes.BTN_NORTH: "x",
    ecodes.BTN_WEST: "y",
    ecodes.BTN_TL: "lb",
    ecodes.BTN_TR: "rb",
    ecodes.BTN_SELECT: "back",
    ecodes.BTN_START: "start",
    ecodes.BTN_MODE: "guide",
    ecodes.BTN_THUMBL: "left_stick_pressed",
    ecodes.BTN_THUMBR: "right_stick_pressed",
}

_AXIS_MAP = {
    ecodes.ABS_X: ("left_stick", "x"),
    ecodes.ABS_Y: ("left_stick", "y"),
    ecodes.ABS_RX: ("right_stick", "x"),
    ecodes.ABS_RY: ("right_stick", "y"),
    ecodes.ABS_Z: ("lt", None),
    ecodes.ABS_RZ: ("rt", None),
    ecodes.ABS_HAT0X: ("dpad", "x"),
    ecodes.ABS_HAT0Y: ("dpad", "y"),
}


def find_xbox_device(name_hint: str = "xbox") -> str:
    """Return the evdev path for the first matching Xbox controller."""
    hint = name_hint.lower()
    for path in list_devices():
        device = InputDevice(path)
        try:
            if hint in device.name.lower():
                return path
        finally:
            device.close()
    raise FileNotFoundError(
        "No Xbox controller found. Is it plugged in? "
        'Check with: python3 -c "from evdev import list_devices; '
        "from evdev import InputDevice; "
        '[print(InputDevice(p).name, p) for p in list_devices()]"'
    )


def _normalize_axis(code: int, value: int, info) -> float:
    """Map raw evdev axis values to floats in [-1, 1] or [0, 1]."""
    if code in (ecodes.ABS_Z, ecodes.ABS_RZ):
        return value / info.max if info.max else 0.0
    if info.min < 0:
        midpoint = (info.min + info.max) / 2.0
        half_range = (info.max - info.min) / 2.0
        return (value - midpoint) / half_range if half_range else 0.0
    return value / info.max if info.max else 0.0


def _hat_to_direction(hat_x: int, hat_y: int) -> DPadDirection:
    if hat_y == -1:
        return DPadDirection.UP
    if hat_y == 1:
        return DPadDirection.DOWN
    if hat_x == -1:
        return DPadDirection.LEFT
    if hat_x == 1:
        return DPadDirection.RIGHT
    return DPadDirection.CENTER


class XboxController(DeviceBase):
    """Poll or iterate over Xbox controller input on Linux.

    Example::

        controller = XboxController()
        while True:
            if controller.poll(timeout=0.1):
                print(controller.state.as_dict())
    """

    def __init__(
        self,
        device_path: Optional[str] = None,
        deadzone: float = 0.08,
        name_hint: str = "xbox",
        start_thread: bool = True,
    ) -> None:
        super().__init__()
        # Command buffers
        self._delta_pos = np.zeros(3)
        self._delta_rot = np.zeros(3)
        self._gripper_value = 0.0
        self._home_flag = False

        path = device_path or find_xbox_device(name_hint=name_hint)
        self.device = InputDevice(path)
        self.deadzone = deadzone
        self.state = XboxControllerState()
        self._abs_info = {code: info for code, info in self.device.capabilities().get(ecodes.EV_ABS, [])}
        self._hat_x = 0
        self._hat_y = 0

        if start_thread:
            self._listener = threading.Thread(target=self.teleop_loop, daemon=True)
            self._listener.start()

    @property
    def name(self) -> str:
        return self.device.name

    @property
    def path(self) -> str:
        return self.device.path

    def close(self) -> None:
        self.device.close()

    def __enter__(self) -> "XboxController":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadzone else value

    def _handle_key(self, code: int, value: int) -> None:
        attr = _BUTTON_MAP.get(code)
        if attr is None:
            return
        if attr == "left_stick_pressed":
            self.state.left_stick.pressed = bool(value)
        elif attr == "right_stick_pressed":
            self.state.right_stick.pressed = bool(value)
        else:
            setattr(self.state, attr, bool(value))

    def _handle_abs(self, code: int, value: int) -> None:
        mapping = _AXIS_MAP.get(code)
        if mapping is None:
            return
        target, sub = mapping
        info = self._abs_info.get(code)
        if info is None:
            return

        if target == "dpad":
            if sub == "x":
                self._hat_x = value
            else:
                self._hat_y = value
            self.state.dpad = _hat_to_direction(self._hat_x, self._hat_y)
            return

        normalized = self._apply_deadzone(_normalize_axis(code, value, info))
        if target == "lt":
            self.state.lt = normalized
        elif target == "rt":
            self.state.rt = normalized
        else:
            stick = getattr(self.state, target)
            setattr(stick, sub, normalized)

    def process_event(self, event) -> None:
        """Apply a single evdev event to the cached state."""
        if event.type == ecodes.EV_KEY:
            self._handle_key(event.code, event.value)
        elif event.type == ecodes.EV_ABS:
            self._handle_abs(event.code, event.value)

    def poll(self, timeout: Optional[float] = None) -> bool:
        """Wait up to ``timeout`` seconds for new events and update state.

        Returns True if any events were processed, False on timeout.
        """
        ready, _, _ = select.select([self.device], [], [], timeout)
        if not ready:
            return False
        for event in self.device.read():
            self.process_event(event)
        return True

    def teleop_loop(self) -> None:
        for _ in self.read_events():
            self._update_control_state()

    def _update_control_state(self) -> None:

        self._delta_pos[0] = -self.state.left_stick.y
        self._delta_pos[1] = -self.state.left_stick.x
        self._delta_pos[2] = -self.state.right_stick.y
        self._gripper_value = self.state.rt
        if self.state.dpad != DPadDirection.CENTER:
            if self.state.dpad == DPadDirection.LEFT:
                self._delta_rot[0] = -1.0
            if self.state.dpad == DPadDirection.RIGHT:
                self._delta_rot[0] = 1.0
            if self.state.dpad == DPadDirection.UP:
                self._delta_rot[1] = -1.0
            if self.state.dpad == DPadDirection.DOWN:
                self._delta_rot[1] = 1.0
        else:
            self._delta_rot[0:2] = 0.0
        self._delta_rot[2] = -self.state.right_stick.x if self.state.rt else 0.0
        self._home_flag = self.state.a

    def read_events(self) -> Iterator:
        """Blocking iterator over raw evdev events (also updates state)."""
        for event in self.device.read_loop():
            self.process_event(event)
            yield event

    def wait_for_button(
        self,
        button: str,
        pressed: bool = True,
        timeout: Optional[float] = None,
    ) -> bool:
        """Block until ``button`` (e.g. ``'a'``, ``'start'``) reaches ``pressed``."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            if not self.poll(timeout=remaining):
                return False
            if getattr(self.state, button, None) == pressed:
                return True

    def on_change(self, callback: Callable[[XboxControllerState], None], timeout: float = 0.05) -> None:
        """Call ``callback`` whenever the controller state changes."""
        last = self.state.as_dict()
        while True:
            if self.poll(timeout=timeout):
                current = self.state.as_dict()
                if current != last:
                    callback(self.state)
                    last = current


if __name__ == "__main__":
    print("Reading controller input. Press Ctrl+C to quit.\n")
    with XboxController(start_thread=False) as controller:
        print(f"Connected: {controller.name} ({controller.path})\n")
        i = 0
        for _ in controller.read_events():
            changed = {
                k: v
                for k, v in controller.state.as_dict().items()
                if v not in (False, 0.0, 0, "center", {"x": 0.0, "y": 0.0, "pressed": False})
            }
            if changed:
                print(changed)
