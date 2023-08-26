## 3dmouse-hid

Use a 3D mouse as a web input device. Great for controlling robot end effectors.

Originally forked from [webhid-space](https://github.com/larsgk/webhid-space). WebHID is only [supported by Chrome](https://caniuse.com/?search=webhid) as of August 2023.

## Development

Run a server in the root of the package:

    python -m http

Open localhost:8000 in Chrome/Chromium.

### Testing with a Fake Robot

To run the interface against a kinematic model of a Franka:

    ros2 launch moveit_servo servo_example.launch.py

Then try `test/test_ros.html`

## Usage


### Identifying Hardware

Running `lsusb` gives us the following:

NOTE:  Using (Ubuntu) Linux, in order to get access from user space, add the following, using the USB Vendor ID returned from `lsusb` to a udev rules file (e.g. `/etc/udev/rules/50-3d-mouse.rules`):

```
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", MODE:="0666", GROUP="input"
```

And run `sudo udevadm control --reload-rules && sudo udevadm trigger`

On other systems, this is not necessary.
