import time
import threading
from typing import Optional

from srl.spacemouse.device import DeviceSpec, SpaceMouseData
from srl.spacemouse.buttons import ButtonState

import numpy as np

TELEOP_CONTROL_RATE = 20

def scale_to_control(x: float, axis_scale: float):
    x = x / axis_scale
    x = min(max(x, -1.0), 1.0)
    return x


def convert(b1, b2, axis_scale: float):
    as_int16 = int.from_bytes([b1,b2], "little", signed=True)
    return scale_to_control(as_int16, axis_scale)


class SpaceMouse:
    def __init__(self, spec: DeviceSpec, control_rate=TELEOP_CONTROL_RATE):

        # Note: these can be found using `hid.enumerate()`
        self.hid_ids = spec.hid_ids
        self.mappings = spec.mappings
        self.button_mapping = spec.button_mapping
        self.axis_scale = spec.axis_scale
        self.name = spec.name
        self._control = None

        # Optional delegate functions that will be called to process/transform position and rotation
        # signal before it is passed out to consumers.
        self._position_callback = None
        self._rotation_callback = None
        self._control_rate = control_rate

        self.thread = None
        self._stop_event = threading.Event()

    def __del__(self):
        if self.is_running:
            self.stop()
        if self.device:
            self.device.close()

    def set_position_callback(self, callback):
        """
        Set a function that will get called to process the raw translation
        readings from the spacemouse. This is useful to remap which direction
        moves which axis (e.g. pushing right moves +y instead of + x). Callback
        should take a single argument (3-dim numpy array for translation) and
        return a 3-dim array as well
        """
        self._position_callback = callback

    def set_rotation_callback(self, callback):
        """
        Set a function that will get called to process the raw RPY readings from the
        spacemouse, before forming an absolute rotation. Callback should take
        3 arguments - roll, pitch, yaw, and return roll, pitch, yaw. This is useful
        for re-mapping the device knob twists to different RPY settings.
        """
        self._rotation_callback = callback

    def get_controller_state(self) -> Optional[SpaceMouseData]:
        """
        Returns the current state of the 3d mouse, a dictionary of pos, orn, and button on/off.
        """
        # Get a copy of the latest data
        control = self._control
        if control is None:
            # The caller must've beaten the actual device thread. No state to give them yet.
            return None
        dpos = np.array(control.xyz)
        rot = np.array(control.rpy)

        # handle callbacks
        if self._position_callback is not None:
            self._position_callback(dpos)

        if self._rotation_callback is not None:
            self._rotation_callback(rot)

        return SpaceMouseData(control.t, dpos, rot, control.buttons)

    @property
    def is_running(self) -> bool:
        return self.thread is not None

    def run(self):
        import hid
        if self.thread:
            return

        # The source for the hid module is a good reference:
        # https://github.com/trezor/cython-hidapi/blob/master/hid.pyx
        self.device = hid.device()
        # self.device.open_path(bytes("/dev/spacemouse", "UTF-8"))
        opened = False
        for vendor_id, product_id in self.hid_ids:
            # Some devices have alternate identifiers. Loop through trying all of them
            try:
                self.device.open(vendor_id, product_id)
                opened = True
                print(f"[spacemouse][device] Successfully connected to: {self.name}, vendor id: { vendor_id }, product id: {product_id}")
            except OSError:
                continue

        if not opened:
            print("Unable to open specified spacemouse device. Ensure you have installed spacenavd, obtained the correct vendor_id and product_id, as well as setting up the correct udev rule and the device is plugged in. ")
            raise RuntimeError("Couldn't open device")
        # We'll use the blocking interface and rely on the timeout feature instead
        # self.device.set_nonblocking(True)

        # launch daemon thread to listen to SpaceNav
        self.thread = threading.Thread(target=self._run_loop)
        self.thread.daemon = True
        self.thread.start()

    def __str__(self) -> str:
        return f"Manufacturer: {self.device.get_manufacturer_string()}\n Product: {self.device.get_product_string()}"

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
        self.device.close()

    def _run_loop(self):
        def state_to_tuple(state):
            return SpaceMouseData(state["t"], np.array((state['x'], state['y'], state['z'])), np.array((state['r'], state['p'], state['ya'])), int(state["buttons"]))
        working_state = {
            "t": -1,
            "x": 0.,
            "y": 0.,
            "z": 0.,
            "r": 0.,
            "p": 0.,
            "ya": 0.,
            "buttons": ButtonState([0] * len(self.button_mapping)),
            "buttons_changed": False,
            "xyz_rpy_change_count": 0,
        }
        self._control = state_to_tuple(working_state)
        while not self._stop_event.is_set():
            d = self.device.read(13, timeout_ms=1000 / self._control_rate)
            if d is not None and len(d) > 0:
                self.process(d, working_state)
                if working_state["xyz_rpy_change_count"] == 2 or working_state["buttons_changed"]:
                    self._control = state_to_tuple(working_state)
                    working_state["xyz_rpy_change_count"] = 0

    def process(self, data, state):
        """
        Update the state based on the incoming data
        This function updates state, giving values for each
        axis [x,y,z,roll,pitch,yaw] in range [-1.0, 1.0]
        The timestamp (in fractional seconds since the start of the program)  is written as element "t"
        If callback is provided, it is called on with a copy of the current state tuple.
        If button_callback is provided, it is called only on button state changes with the argument (state, button_state).
        Parameters:
            data    The data for this HID event, as returned by the HID callback
        """

        for name, (chan, b1, b2, flip) in self.mappings.items():
            if data[0] == chan:
                state[name] = (
                    flip * convert(data[b1], data[b2], self.axis_scale)
                )
                if name == "r" or name == "x":
                    state["xyz_rpy_change_count"] += 1

        for button_index, (_, chan, byte, bit) in enumerate(self.button_mapping):
            if data[0] == chan:
                state["buttons_changed"] = True
                # update the button vector
                mask = 1 << bit
                state["buttons"][button_index] = (
                    1 if (data[byte] & mask) != 0 else 0
                )

        state["t"] = time.time()
