import time
import threading
from collections import namedtuple
from typing import Dict, List

import numpy as np

TELEOP_CONTROL_RATE = 20

DOUBLE_CLICK_TIME = 0.3


# tuple for 6DOF results
SpaceMouseData = namedtuple(
    "SpaceMouseData", ["t", "xyz", "rpy", "buttons"]
)


class ButtonState(list):
    def __int__(self):
        # Button state is a list of bools, so convert to a compact int bitfield representation
        return sum((b << i) for (i, b) in enumerate(self))


class ButtonStateStruct:
    def __init__(self, value: int, name_to_index: Dict[str, int]):
        """
        Args:
            value (int): the packed bitfield representation of the button state
            mapping (List[str]): name of each index in the bitfield
        """
        self.value = value
        self.name_to_index = name_to_index

    def __getitem__(self, name: str) -> bool:
        if name not in self.name_to_index.keys():
            return False
        index = self.name_to_index[name]
        return (self.value >> index) & 1


# axis mappings are specified as:
# [channel, byte1, byte2, scale]; scale is usually just -1 or 1 and multiplies the result by this value
# (but per-axis scaling can also be achieved by setting this value)
# byte1 and byte2 are indices into the HID array indicating the two bytes to read to form the value for this axis
# For the SpaceNavigator, these are consecutive bytes following the channel number.
AxisSpec = namedtuple("AxisSpec", ["channel", "byte1", "byte2", "scale"])

# button states are specified as:
# [channel, data byte,  bit of byte, index to write to]
# If a message is received on the specified channel, the value of the data byte is set in the button bit array
ButtonSpec = namedtuple("ButtonSpec", ["name", "channel", "byte", "bit"])
DeviceSpec = namedtuple("DeviceSpec", ["name", "hid_ids", "led_id", "mappings", "button_mapping", "axis_scale"])


# the IDs for the supported devices
# Each ID maps a device name to a DeviceSpec object
DEVICE_SPECS = {
    "SpaceNavigator": DeviceSpec(
        name="SpaceNavigator",
        hid_ids=[[0x46D, 0xC626]],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "p": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "r": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "ya": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(name="LEFT", channel=3, byte=1, bit=0),
            ButtonSpec(name="RIGHT", channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Compact": DeviceSpec(
        name="SpaceMouse Compact",
        hid_ids=[[0x256F, 0xC635]],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "p": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "r": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "ya": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(name="LEFT", channel=3, byte=1, bit=0),
            ButtonSpec(name="RIGHT", channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Pro Wireless": DeviceSpec(
        name="SpaceMouse Pro Wireless",
        hid_ids=[[0x256F, 0xC632],[0x256F, 0xC631]],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "p": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "r": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "ya": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(name="MENU", channel=3, byte=1, bit=0),
            ButtonSpec(name="ALT", channel=3, byte=3, bit=7),
            ButtonSpec(name="CTRL", channel=3, byte=4, bit=1),
            ButtonSpec(name="SHIFT", channel=3, byte=4, bit=0),
            ButtonSpec(name="ESC", channel=3, byte=3, bit=6),
            ButtonSpec(name="1", channel=3, byte=2, bit=4),
            ButtonSpec(name="2", channel=3, byte=2, bit=5),
            ButtonSpec(name="3", channel=3, byte=2, bit=6),
            ButtonSpec(name="4", channel=3, byte=2, bit=7),
            ButtonSpec(name="ROLL CLOCKWISE", channel=3, byte=2, bit=0),
            ButtonSpec(name="T", channel=3, byte=1, bit=2), # Top, Front, Right, but we'll leave them abbreviated to not conflict with Compact's buttons
            ButtonSpec(name="ROTATION", channel=3, byte=4, bit=2),
            ButtonSpec(name="F", channel=3, byte=1, bit=5),
            ButtonSpec(name="R", channel=3, byte=1, bit=4),
            ButtonSpec(name="FIT", channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Pro": DeviceSpec(
        name="SpaceMouse Pro",
        hid_ids=[[0x46D, 0xC62b]],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "p": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "r": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "ya": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(name="MENU", channel=3, byte=1, bit=0),
            ButtonSpec(name="ALT", channel=3, byte=3, bit=7),
            ButtonSpec(name="CTRL", channel=3, byte=4, bit=1),
            ButtonSpec(name="SHIFT", channel=3, byte=4, bit=0),
            ButtonSpec(name="ESC", channel=3, byte=3, bit=6),
            ButtonSpec(name="1", channel=3, byte=2, bit=4),
            ButtonSpec(name="2", channel=3, byte=2, bit=5),
            ButtonSpec(name="3", channel=3, byte=2, bit=6),
            ButtonSpec(name="4", channel=3, byte=2, bit=7),
            ButtonSpec(name="ROLL CLOCKWISE", channel=3, byte=2, bit=0),
            ButtonSpec(name="T", channel=3, byte=1, bit=2),
            ButtonSpec(name="ROTATION", channel=3, byte=4, bit=2),
            ButtonSpec(name="F", channel=3, byte=1, bit=5),
            ButtonSpec(name="R", channel=3, byte=1, bit=4),
            ButtonSpec(name="FIT", channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Wireless": DeviceSpec(
        name="SpaceMouse Wireless",
        hid_ids=[[0x256F, 0xC62E], [0x256F, 0xC652]],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "p": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "r": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "ya": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(name="LEFT", channel=3, byte=1, bit=0),
            ButtonSpec(name="RIGHT", channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "3Dconnexion Universal Receiver": DeviceSpec(
        name="3Dconnexion Universal Receiver",
        hid_ids=[[0x256F, 0xC652]],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "p": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "r": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "ya": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(name="MENU", channel=3, byte=1, bit=0),
            ButtonSpec(name="ALT", channel=3, byte=3, bit=7),
            ButtonSpec(name="CTRL", channel=3, byte=4, bit=1),
            ButtonSpec(name="SHIFT", channel=3, byte=4, bit=0),
            ButtonSpec(name="ESC", channel=3, byte=3, bit=6),
            ButtonSpec(name="1", channel=3, byte=2, bit=4),
            ButtonSpec(name="2", channel=3, byte=2, bit=5),
            ButtonSpec(name="3", channel=3, byte=2, bit=6),
            ButtonSpec(name="4", channel=3, byte=2, bit=7),
            ButtonSpec(name="ROLL CLOCKWISE", channel=3, byte=2, bit=0),
            ButtonSpec(name="T", channel=3, byte=1, bit=2),
            ButtonSpec(name="ROTATION", channel=3, byte=4, bit=2),
            ButtonSpec(name="F", channel=3, byte=1, bit=5),
            ButtonSpec(name="R", channel=3, byte=1, bit=4),
            ButtonSpec(name="FIT", channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
}

DEVICE_NAMES = list(DEVICE_SPECS.keys())

DEVICE_BUTTON_STRUCT_INDICES: Dict[str, Dict[str, int]] = {}
for device_name, spec in DEVICE_SPECS.items():
    # We use the order that the buttons are listed in the spec to define the packing arrangement for a bitfield representation
    # of button state.
    DEVICE_BUTTON_STRUCT_INDICES[device_name] = {name: index for index, (name, _, _, _) in enumerate(spec.button_mapping)}


def scale_to_control(x, axis_scale):
    x = x / axis_scale
    x = min(max(x, -1.0), 1.0)
    return x


def convert(b1, b2, axis_scale):
    as_int16 = int.from_bytes([b1,b2], "little", signed=True)
    return scale_to_control(as_int16, axis_scale)


class SpaceMouse(object):
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

    def get_controller_state(self) -> SpaceMouseData:
        """
        Returns the current state of the 3d mouse, a dictionary of pos, orn, and button on/off.
        """
        # Get a copy of the latest data
        control = self._control
        dpos = np.array(control.xyz)
        rot = np.array(control.rpy)

        # handle callbacks
        if self._position_callback is not None:
            self._position_callback(dpos)

        if self._rotation_callback is not None:
            self._rotation_callback(rot)

        return SpaceMouseData(control.t, dpos, rot, control.buttons)

    @property
    def is_running(self):
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
                print(f"[spacemouse][device] Successfully connected to: {spec.name}, vendor id: { vendor_id }, product id: {product_id}")
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