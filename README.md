## 3dmouse-hid

Use a 3D mouse as a web input device. Great for controlling robot end effectors.

Originally forked from [webhid-space](https://github.com/larsgk/webhid-space), with device support information taken from [pyspacenavigator](https://github.com/johnhw/pyspacenavigator).

## Usage

We have tested this code under Ubuntu 22.04. Note that WebHID is only [supported by Chrome and Edge](https://caniuse.com/?search=webhid) as of October 2023. You may experience issues using WebHID with Chromium in recent Ubuntu releases due to Snap isolation.

### Connecting a 3D Mouse

When using (Ubuntu) Linux, you need to give your user permission to access the device. Add a udev rule using the USB Vendor ID returned from `lsusb` (e.g. `/etc/udev/rules.d/50-3d-mouse.rules`):

```
# Check the Vendor ID using `lsusb`! See the full set of possible values in `dist/deviceSpecs.js`
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", MODE:="0666", GROUP="input"
```

And run `sudo udevadm control --reload-rules && sudo udevadm trigger`

On other systems, this is not necessary.

### Running the Included Demos

No build steps are required to run our included demos. Simply run a server using:

    npm run test

Open localhost:8000/test in Chrome. Note WebHID is typically only available in HTTPS contexts.

#### Playing with the Visualization and Filters

You can test out the twist visualization using `viz_test.html` by opening `http://localhost:8000/test/viz_test.html`.

#### Testing with a Simulated UR5

We include a ROS2 Humble robot simulation to enable testing without a robot. You will need to install various UR ROS2 packages to use it:

    sudo apt install ros-humble-ur-description ros-humble-controller-manager ros-humble-ur-moveit-config ros-humble-ur-simulation-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-image ros-humble-rosbridge-suite ros-humble-ros2-control ros-humble-position-controllers ros-humble-image-transport ros-humble-image-transport-plugins

Our main launch file calls launch files [created by Universal Robotics](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation) for placing their robot model in a Gazebo simulation, and also homes the robot and configures camera views:

    cd 3dmouse-hid/test/ros && ros2 launch ur_teleop_sim.launch.py

This launch runs headless by default. [MoveIt 2 Servo](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html), the package which provides the end-effector twist controller, is initialized and activated in the final step of the launch file, after which you can confirm that servo is running:

    ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{ header: { stamp: 'now', 'frame_id': 'tool0' },  twist: {linear: {x: -0.1}, angular: {  }}}" -r 10

Now you should be able to open `http://localhost:8000/test/` and follow the interface instructions to teleoperate the simulated robot.

### Debugging WebHID Issues

Try using nondebug's [WebHID Explorer](https://nondebug.github.io/webhid-explorer/) to connect to the device. If the connection fails, take a look at [chrome://device-log/](chrome://device-log/) to see why.
