/**
 * Factory function to create a named tuple-like constructor.
 * Similar to Python's namedtuple, but in JavaScript flavor.
 *
 * @param {string} name - The name of the tuple (primarily for debugging purposes).
 * @param {string[]} properties - An array of property names for the tuple.
 * @returns {function} - A constructor function for the tuple.
 */

function createNamedTuple(name, properties) {
    return function (...values) {
        const tuple = {};
        properties.forEach((property, index) => {
            tuple[property] = values[index];
        });
        return tuple;
    };
}


/**
 * Device Mapping Configuration:
 *
 * This section is dedicated to defining and mapping various devices.
 * It ensures that the +X axis extends out the right of the puck,
 * +Y out the front, and +Z upward. Roll, pitch, and yaw are configured
 * so that positive rotations correspond to clockwise motion when looking
 * along the positive axis. When mapped correctly, the TwistViz should
 * appear to mimic the movement applied to the device.
 */


const AxisSpec = createNamedTuple("AxisSpec", ["channel", "byte", "scale"]);
const ButtonSpec = createNamedTuple("ButtonSpec", ["name", "channel", "byte", "bit"]);
const DeviceSpec = createNamedTuple("DeviceSpec", ["name", "hidIds", "ledId", "mappings", "buttonMapping", "axisScale"]);

/**
 * Detailed specifications for each supported device.
 * This object acts as a dictionary where the key is the device name,
 * and the value is an instance of DeviceSpec, detailing its configuration.
 */

export const DEVICE_SPECS = {
    "SpaceNavigator": new DeviceSpec(
        "SpaceNavigator",
        [[0x46D, 0xC626]],
        [0x8, 0x4B],
        {
            "x": new AxisSpec(1, 0, 1),
            "y": new AxisSpec(1, 2, -1),
            "z": new AxisSpec(1, 4, -1),
            "roll": new AxisSpec(2, 0, -1),
            "pitch": new AxisSpec(2, 2, -1),
            "yaw": new AxisSpec(2, 4, 1),
        },
        [
            new ButtonSpec("LEFT", 3, 0, 0),
            new ButtonSpec("RIGHT", 3, 0, 1),
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
            "roll": new AxisSpec(2, 0, -1),
            "pitch": new AxisSpec(2, 2, -1),
            "yaw": new AxisSpec(2, 4, 1),
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
            "roll": new AxisSpec(1, 6, -1),
            "pitch": new AxisSpec(1, 8, -1),
            "yaw": new AxisSpec(1, 10, 1),
        },
        [
            new ButtonSpec("MENU", 3, 0, 0),
            new ButtonSpec("ALT", 3, 2, 7),
            new ButtonSpec("CTRL", 3, 3, 1),
            new ButtonSpec("SHIFT", 3, 3, 0),
            new ButtonSpec("ESC", 3, 2, 6),
            new ButtonSpec("1", 3, 1, 4),
            new ButtonSpec("2", 3, 1, 5),
            new ButtonSpec("3", 3, 1, 6),
            new ButtonSpec("4", 3, 1, 7),
            new ButtonSpec("ROLL CLOCKWISE", 3, 1, 0),
            new ButtonSpec("T", 3, 0, 2),
            new ButtonSpec("ROTATION", 3, 3, 2),
            new ButtonSpec("F", 3, 0, 5),
            new ButtonSpec("R", 3, 0, 4),
            new ButtonSpec("FIT", 3, 0, 1),
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
            "roll": new AxisSpec(2, 0, 1),
            "pitch": new AxisSpec(2, 2, -1),
            "yaw": new AxisSpec(2, 4, -1),
        },
        [
            new ButtonSpec("MENU", 3, 0, 0),
            new ButtonSpec("ALT", 3, 2, 7),
            new ButtonSpec("CTRL", 3, 3, 1),
            new ButtonSpec("SHIFT", 3, 3, 0),
            new ButtonSpec("ESC", 3, 2, 6),
            new ButtonSpec("1", 3, 1, 4),
            new ButtonSpec("2", 3, 1, 5),
            new ButtonSpec("3", 3, 1, 6),
            new ButtonSpec("4", 3, 1, 7),
            new ButtonSpec("ROLL CLOCKWISE", 3, 1, 0),
            new ButtonSpec("T", 3, 0, 2),
            new ButtonSpec("ROTATION", 3, 3, 2),
            new ButtonSpec("F", 3, 0, 5),
            new ButtonSpec("R", 3, 0, 4),
            new ButtonSpec("FIT", 3, 0, 1),
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
            "roll": new AxisSpec(1, 6, -1),
            "pitch": new AxisSpec(1, 8, -1),
            "yaw": new AxisSpec(1, 10, 1),
        },
        [
            new ButtonSpec("LEFT", 3, 0, 0),
            new ButtonSpec("RIGHT", 3, 0, 1),
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
            "roll": new AxisSpec(1, 6, -1),
            "pitch": new AxisSpec(1, 8, -1),
            "yaw": new AxisSpec(1, 10, 1),
        },
        [
            new ButtonSpec("MENU", 3, 0, 0),
            new ButtonSpec("ALT", 3, 2, 7),
            new ButtonSpec("CTRL", 3, 3, 1),
            new ButtonSpec("SHIFT", 3, 3, 0),
            new ButtonSpec("ESC", 3, 2, 6),
            new ButtonSpec("1", 3, 1, 4),
            new ButtonSpec("2", 3, 1, 5),
            new ButtonSpec("3", 3, 1, 6),
            new ButtonSpec("4", 3, 1, 7),
            new ButtonSpec("ROLL CLOCKWISE", 3, 1, 0),
            new ButtonSpec("T", 3, 0, 2),
            new ButtonSpec("ROTATION", 3, 3, 2),
            new ButtonSpec("F", 3, 0, 5),
            new ButtonSpec("R", 3, 0, 4),
            new ButtonSpec("FIT", 3, 0, 1),
        ],
        350.0
    ),
};

/**
 * Generates a list of HID filters based on the device specifications.
 * These filters can be used to request access to specific HID devices.
 */

let filters = [];
for (const properties of Object.values(DEVICE_SPECS)) {
    for (const [vendorId, productId] of properties.hidIds) {
        filters.push({vendorId: vendorId, productId: productId});
    }
}

export const HID_FILTERS = filters;

/**
 * Maps vendor and product IDs to device names.
 * This dictionary can be used to identify devices based on their IDs.
 */


let idsToName = {}
for (const properties of Object.values(DEVICE_SPECS)) {
    for (const [vendorId, productId] of properties.hidIds) {
        idsToName[[vendorId, productId]] = properties.name;
    }
}

export const IDS_TO_NAME = idsToName;