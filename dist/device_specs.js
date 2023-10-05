// Similar to Python's namedtuple
function createNamedTuple(name, properties) {
  return function (...values) {
    const tuple = {};
    properties.forEach((property, index) => {
      tuple[property] = values[index];
    });
    return tuple;
  };
}

const AxisSpec = createNamedTuple("AxisSpec", ["channel", "byte", "scale"]);
const ButtonSpec = createNamedTuple("ButtonSpec", ["name", "channel", "byte", "bit"]);
const DeviceSpec = createNamedTuple("DeviceSpec", ["name", "hidIds", "ledId", "mappings", "buttonMapping", "axisScale"]);

export const DEVICE_SPECS = {
  "SpaceNavigator": new DeviceSpec(
    "SpaceNavigator",
    [[0x46D, 0xC626]],
    [0x8, 0x4B],
    {
      "x": new AxisSpec(1, 0, 1),
      "y": new AxisSpec(1, 2, -1),
      "z": new AxisSpec(1, 4, -1),
      "p": new AxisSpec(2, 0, -1),
      "r": new AxisSpec(2, 2, -1),
      "ya": new AxisSpec(2, 4, 1),
    },
    [
      new ButtonSpec("LEFT", 3, 1, 0),
      new ButtonSpec("RIGHT", 3, 1, 1),
    ],
    350.0
  ),
  "SpaceMouse Compact": new DeviceSpec(
    "SpaceMouse Compact",
    [[0x256F, 0xC635]],
    [0x8, 0x4B],
    {
      "x": new AxisSpec(1, 0, 1),
      "y": new AxisSpec(1, 2, -1),
      "z": new AxisSpec(1, 4, -1),
      "p": new AxisSpec(2, 0, -1),
      "r": new AxisSpec(2, 2, -1),
      "ya": new AxisSpec(2, 4, 1),
    },
    [
      new ButtonSpec("LEFT", 3, 1, 0),
      new ButtonSpec("RIGHT", 3, 1, 1),
    ],
    350.0
  ),
  "SpaceMouse Pro Wireless": new DeviceSpec(
    "SpaceMouse Pro Wireless",
    [[0x256F, 0xC632], [0x256F, 0xC631]],
    [0x8, 0x4B],
    {
      "x": new AxisSpec(1, 0, 1),
      "y": new AxisSpec(1, 2, -1),
      "z": new AxisSpec(1, 4, -1),
      "p": new AxisSpec(1, 6, -1),
      "r": new AxisSpec(1, 8, -1),
      "ya": new AxisSpec(1, 10, 1),
    },
    [
      new ButtonSpec("MENU", 3, 1, 0),
      new ButtonSpec("ALT", 3, 3, 7),
      new ButtonSpec("CTRL", 3, 4, 1),
      new ButtonSpec("SHIFT", 3, 4, 0),
      new ButtonSpec("ESC", 3, 3, 6),
      new ButtonSpec("1", 3, 2, 4),
      new ButtonSpec("2", 3, 2, 5),
      new ButtonSpec("3", 3, 2, 6),
      new ButtonSpec("4", 3, 2, 7),
      new ButtonSpec("ROLL CLOCKWISE", 3, 2, 0),
      new ButtonSpec("T", 3, 1, 2),
      new ButtonSpec("ROTATION", 3, 4, 2),
      new ButtonSpec("F", 3, 1, 5),
      new ButtonSpec("R", 3, 1, 4),
      new ButtonSpec("FIT", 3, 1, 1),
    ],
    350.0
  ),
  "SpaceMouse Pro": new DeviceSpec(
    "SpaceMouse Pro",
    [[0x46D, 0xC62b]],
    [0x8, 0x4B],
    {
      "x": new AxisSpec(1, 0, 1),
      "y": new AxisSpec(1, 2, -1),
      "z": new AxisSpec(1, 4, -1),
      "p": new AxisSpec(2, 0, -1),
      "r": new AxisSpec(2, 2, -1),
      "ya": new AxisSpec(2, 4, 1),
    },
    [
      new ButtonSpec("MENU", 3, 1, 0),
      new ButtonSpec("ALT", 3, 3, 7),
      new ButtonSpec("CTRL", 3, 4, 1),
      new ButtonSpec("SHIFT", 3, 4, 0),
      new ButtonSpec("ESC", 3, 3, 6),
      new ButtonSpec("1", 3, 2, 4),
      new ButtonSpec("2", 3, 2, 5),
      new ButtonSpec("3", 3, 2, 6),
      new ButtonSpec("4", 3, 2, 7),
      new ButtonSpec("ROLL CLOCKWISE", 3, 2, 0),
      new ButtonSpec("T", 3, 1, 2),
      new ButtonSpec("ROTATION", 3, 4, 2),
      new ButtonSpec("F", 3, 1, 5),
      new ButtonSpec("R", 3, 1, 4),
      new ButtonSpec("FIT", 3, 1, 1),
    ],
    350.0
  ),
  "SpaceMouse Wireless": new DeviceSpec(
    "SpaceMouse Wireless",
    [[0x256F, 0xC62E], [0x256F, 0xC652]],
    [0x8, 0x4B],
    {
      "x": new AxisSpec(1, 0, 1),
      "y": new AxisSpec(1, 2, -1),
      "z": new AxisSpec(1, 4, -1),
      "p": new AxisSpec(1, 6, -1),
      "r": new AxisSpec(1, 8, -1),
      "ya": new AxisSpec(1, 10, 1),
    },
    [
      new ButtonSpec("LEFT", 3, 1, 0),
      new ButtonSpec("RIGHT", 3, 1, 1),
    ],
    350.0
  ),
  "3Dconnexion Universal Receiver": new DeviceSpec(
    "3Dconnexion Universal Receiver",
    [[0x256F, 0xC652]],
    [0x8, 0x4B],
    {
      "x": new AxisSpec(1, 0, 1),
      "y": new AxisSpec(1, 2, -1),
      "z": new AxisSpec(1, 4, -1),
      "p": new AxisSpec(1, 6, -1),
      "r": new AxisSpec(1, 8, -1),
      "ya": new AxisSpec(1, 10, 1),
    },
    [
      new ButtonSpec("MENU", 3, 1, 0),
      new ButtonSpec("ALT", 3, 3, 7),
      new ButtonSpec("CTRL", 3, 4, 1),
      new ButtonSpec("SHIFT", 3, 4, 0),
      new ButtonSpec("ESC", 3, 3, 6),
      new ButtonSpec("1", 3, 2, 4),
      new ButtonSpec("2", 3, 2, 5),
      new ButtonSpec("3", 3, 2, 6),
      new ButtonSpec("4", 3, 2, 7),
      new ButtonSpec("ROLL CLOCKWISE", 3, 2, 0),
      new ButtonSpec("T", 3, 1, 2),
      new ButtonSpec("ROTATION", 3, 4, 2),
      new ButtonSpec("F", 3, 1, 5),
      new ButtonSpec("R", 3, 1, 4),
      new ButtonSpec("FIT", 3, 1, 1),
    ],
    350.0
  ),
};


let filters = [];
for (const [_, properties] of Object.entries(DEVICE_SPECS)) {
    for (const [vendorId, productId] of properties.hidIds) {
        filters.push({ vendorId: vendorId, productId: productId });
    }
}

export const HID_FILTERS = filters;

let idsToName = {}
for (const [_, properties] of Object.entries(DEVICE_SPECS)) {
  for (const [vendorId, productId] of properties.hidIds) {
      idsToName[[vendorId, productId]] = properties.name;
  }
}

export const IDS_TO_NAME = idsToName;