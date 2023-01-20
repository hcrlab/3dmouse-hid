from collections import namedtuple


SpaceMouseData = namedtuple(
    "SpaceMouseData", ["t", "xyz", "rpy", "buttons"]
)

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
