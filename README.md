## 3dmouse-hid

Use a 3D mouse as a web input device. Great for controlling robot end effectors.

Originally forked from [webhid-space](https://github.com/larsgk/webhid-space). WebHID is only [supported by Chrome and Edge](https://caniuse.com/?search=webhid) as of August 2023. You may experience issues using WebHID with Chromium in recend Ubuntu releases due to Snap isolation.

## Development

Run a server in the root of the package:

   python3 -m http.server --bind 127.0.0.1


Open localhost:8000 in Chrome. Note WebHID is typically only available in HTTPS contexts, and that the default address, `0.0.0.0` won't work as it isn't allowlisted.

### Debugging WebHID Device Connections

Try using nondebug's [WebHID Explorer](https://nondebug.github.io/webhid-explorer/) to connect to the device. If the connection fails, take a look at [chrome://device-log/](chrome://device-log/) to see why.

### Testing with a Fake Robot

To run the interface against a kinematic model of a Franka:

    ros2 launch moveit_servo servo_example.launch.py

Then try `test/test_ros.html`

## Usage


Using (Ubuntu) Linux, in order to get access from user space, add the following, using the USB Vendor ID returned from `lsusb` to a udev rules file (e.g. `/etc/udev/rules/50-3d-mouse.rules`):

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", MODE:="0666", GROUP="input"
```

And run `sudo udevadm control --reload-rules && sudo udevadm trigger`

On other systems, this is not necessary.
