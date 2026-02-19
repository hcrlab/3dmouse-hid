import logging
import threading
import time
from typing import Optional

import numpy as np
import pyspacemouse
from pyspacemouse import DeviceInfo

from threed_mouse.device import SpaceMouseData

logger = logging.getLogger(__name__)

# Control rate (Hz) for reading from the device in the background thread.
TELEOP_CONTROL_RATE = 20


class SpaceMouse:
    def __init__(
        self,
        spec: Optional[DeviceInfo] = None,
        control_rate=TELEOP_CONTROL_RATE,
        device: Optional[str] = None,
        device_index: int = 0,
    ):
        # If both are provided, explicit device name wins.
        self.name = device if device is not None else (spec.name if spec is not None else None)
        self._control_rate = control_rate
        self._device_index = device_index

        # Optional delegate functions that process position/rotation before
        # data is surfaced to callers.
        self._position_callback = None
        self._rotation_callback = None
        self._unexpected_close_callback = None

        self._control: Optional[SpaceMouseData] = None
        self._control_lock = threading.Lock()
        self._frame_rotation_linear = np.eye(3, dtype=float)
        self._frame_rotation_angular = np.eye(3, dtype=float)

        self.device = None
        self.thread = None
        self._stop_event = threading.Event()

    def __del__(self):
        if self.is_running:
            self.stop()
        if self.device:
            self.device.close()

    def set_position_callback(self, callback):
        self._position_callback = callback

    def set_rotation_callback(self, callback):
        self._rotation_callback = callback

    def set_unexpected_close_callback(self, callback):
        self._unexpected_close_callback = callback

    def _validate_frame_rotation(
        self,
        rotation: np.ndarray,
        *,
        require_right_handed: bool,
    ) -> np.ndarray:
        r = np.asarray(rotation, dtype=float)
        if r.shape != (3, 3):
            raise ValueError("rotation must have shape (3, 3)")

        if not np.allclose(r.T @ r, np.eye(3), atol=1e-6):
            raise ValueError("rotation must be orthonormal (R.T @ R == I)")

        if require_right_handed:
            det = np.linalg.det(r)
            if not np.isclose(det, 1.0, atol=1e-6):
                raise ValueError("rotation must be right-handed (det(R) == +1)")

        return r

    def set_frame_rotation(self, rotation: np.ndarray) -> None:
        """Set one frame matrix for both translation and rotation."""
        r = self._validate_frame_rotation(rotation, require_right_handed=True)
        with self._control_lock:
            self._frame_rotation_linear = r
            self._frame_rotation_angular = r

    def set_frame_rotations(
        self,
        linear_rotation: np.ndarray,
        angular_rotation: np.ndarray,
        *,
        allow_angular_reflection: bool = True,
    ) -> None:
        """Set separate frame matrices for translation and rotation.

        `linear_rotation` is required to be a proper rotation (det=+1).
        `angular_rotation` can optionally allow det=-1 for device quirks.
        """
        r_linear = self._validate_frame_rotation(
            linear_rotation,
            require_right_handed=True,
        )
        r_angular = self._validate_frame_rotation(
            angular_rotation,
            require_right_handed=not allow_angular_reflection,
        )
        with self._control_lock:
            self._frame_rotation_linear = r_linear
            self._frame_rotation_angular = r_angular

    def get_controller_state(self) -> Optional[SpaceMouseData]:
        with self._control_lock:
            control = self._control
        if control is None:
            return None

        dpos = np.array(control.xyz, dtype=float)
        rot = np.array(control.rpy, dtype=float)

        if self._position_callback is not None:
            self._position_callback(dpos)
        if self._rotation_callback is not None:
            self._rotation_callback(rot)

        buttons = np.asarray(control.buttons, dtype=int).copy()
        return SpaceMouseData(control.t, dpos, rot, buttons)

    @property
    def is_running(self) -> bool:
        return self.thread is not None

    def run(self):
        if self.thread:
            return

        try:
            self.device = pyspacemouse.open(
                device=self.name,
                device_index=self._device_index,
                nonblocking=True,
            )
            # Capture resolved device name after pyspacemouse autodiscovery.
            self.name = self.device.name
        except Exception as e:
            logger.error("Unable to open SpaceMouse '%s': %s", self.name, e)
            raise RuntimeError("Couldn't open device") from e

        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def __str__(self) -> str:
        if self.device is None:
            return "No connected SpaceMouse device."
        return self.device.describe_connection()

    def stop(self):
        if not self.is_running:
            return
        self._stop_event.set()
        if self.thread:
            self.thread.join()
        self._stop_event.clear()
        self.thread = None

    def close(self):
        self.stop()
        if self.device is not None:
            self.device.close()
            self.device = None

    def _run_loop(self):
        last_t = None
        sleep_s = max(0.001, 1.0 / float(self._control_rate))

        # Initialize with a neutral state so consumers can poll immediately.
        with self._control_lock:
            self._control = SpaceMouseData(
                -1.0,
                np.array((0.0, 0.0, 0.0), dtype=float),
                np.array((0.0, 0.0, 0.0), dtype=float),
                np.zeros((0,), dtype=int),
            )

        while not self._stop_event.is_set():
            try:
                state = self.device.read()
            except Exception:
                logger.warning("Lost connection to SpaceMouse. Closing device.")
                if self.device is not None:
                    self.device.close()
                    self.device = None
                if self._unexpected_close_callback:
                    self._unexpected_close_callback()
                self.thread = None
                break

            if state.t != last_t:
                last_t = state.t
                with self._control_lock:
                    r_linear = self._frame_rotation_linear.copy()
                    r_angular = self._frame_rotation_angular.copy()
                xyz = r_linear @ np.array((state.x, state.y, state.z), dtype=float)
                rpy = r_angular @ np.array((state.roll, state.pitch, state.yaw), dtype=float)
                control = SpaceMouseData(
                    state.t,
                    xyz,
                    rpy,
                    np.asarray(state.buttons, dtype=int),
                )
                with self._control_lock:
                    self._control = control

            time.sleep(sleep_s)
