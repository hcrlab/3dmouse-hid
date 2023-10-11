## 3dmouse-hid

Use a 3D mouse as a web input device. Great for controlling robot end effectors.

Originally forked from [webhid-space](https://github.com/larsgk/webhid-space). WebHID is only [supported by Chrome and Edge](https://caniuse.com/?search=webhid) as of August 2023. You may experience issues using WebHID with Chromium in recent Ubuntu releases due to Snap isolation.

## Development

Run a server in the root of the package:

   python3 -m http.server --bind 127.0.0.1


Open localhost:8000 in Chrome. Note WebHID is typically only available in HTTPS contexts, and that the default address, `0.0.0.0` won't work as it isn't allowlisted.

### Debugging WebHID Device Connections

Try using nondebug's [WebHID Explorer](https://nondebug.github.io/webhid-explorer/) to connect to the device. If the connection fails, take a look at [chrome://device-log/](chrome://device-log/) to see why.

### Testing with a Fake Robot

To run the interface against a kinematic model of a Franka:

    ros2 launch moveit_servo servo_example.launch.py

Then try `test/index.html`

### Testing with a Simulated UR5

    # Make a workspace, put the UR gazebo sim in it
    git clone git@github.com:UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git

    # You must manually add "use_fake_hardware" : "true" as specified in this PR: https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/pull/22 . The robot will not move unless you do tihs.
    colcon build

    rosdep install --from-paths .
    sudo apt install ros-humble-ur-description ros-humble-controller-manager ros-humble-ur-moveit-config sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-webbridge-suite ros-humble-ros2-control ros-humble-position-controllers


    # ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{ header: { stamp: 'now', 'frame_id': 'tool0' },  twist: {linear: {x: -0.1}, angular: {  }}}" -r 10


    ros2 launch 3dmouse-hid/test/ros/ur_teleop_sim.launch.py


## Usage


Using (Ubuntu) Linux, in order to get access from user space, add the following, using the USB Vendor ID returned from `lsusb` to a udev rules file (e.g. `/etc/udev/rules/50-3d-mouse.rules`):

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", MODE:="0666", GROUP="input"
```

And run `sudo udevadm control --reload-rules && sudo udevadm trigger`

On other systems, this is not necessary.
