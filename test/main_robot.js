import * as SpaceDriver from '../dist/space-driver.js';
import { ThreeDMouse } from '../dist/space-driver.js';


// Configure the callback

var ros=new ROSLIB.Ros({
    url : 'ws://127.0.0.1:9090'
});
 
// Event handler for successful connection
ros.on('connection', function() {
    document.getElementById("status").innerHTML = "Connected";
});

// Event handler for connection errors
ros.on('error', function(error) {
    document.getElementById("status").innerHTML = "Error";
});

// Event handler for connection closure
ros.on('close', function() {
    document.getElementById("status").innerHTML = "Closed";
});

// Assuming 'robotTopic' is your ROS 2 topic for publishing
var move_robotTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/servo_node/delta_twist_cmds', // Replace with your topic name
    messageType: 'geometry_msgs/msg/TwistStamped' // Specify the correct message type
});

export function move_robot(report) {
    // Assuming you have a ROS node initialized as 'rosNode'

    // Create a Twist message
    var twist = new ROSLIB.Message({
        twist: {
            linear: {
                x: report.Tx_f,
                y: report.Ty_f,
                z: report.Tz_f
            },
            angular: {
                x: -report.Rx_f,
                y: report.Ry_f,
                z: -report.Rz_f
            }
        }
    });
    var header = new ROSLIB.Message({
        stamp: {
            sec: Math.floor(Date.now() / 1000), // Current time in seconds
            nanosec: (Date.now() % 1000) * 1e6 // Current time in nanoseconds
        },
        frame_id: 'panda_hand' // Replace with your desired frame_id
    });

    // Assign the header to the TwistStamped message
    twist.header = header;

    

    // Publish the Twist message to the topic
    move_robotTopic.publish(twist);
    

    console.log("Published Twist message to robotTopic.");
}
 
function openGripper() {
    const actionClient = new ROSLIB.ActionClient({
      ros: ros,
      serverName: '/panda_hand_controller/gripper_cmd', // Replace with your action server name
      actionName: 'control_msgs/GripperCommandAction ' // Replace with your action name
    });

    const goal = new ROSLIB.Goal({
      actionClient: actionClient,
      goalMessage: {
        command: {
          position: 0.04,
          max_effort: 2.0
        }
      }
    });

    goal.on('feedback', function (feedback) {
      console.log('Feedback:', feedback);
    });

    goal.on('result', function (result) {
      console.log('Final Result:', result);
    });

    console.log('Sending open gripper goal...');
    goal.send();

    goal.on('result', function (result) {
      console.log('Open gripper action completed with result:', result);
    });
  }

  function closeGripper() {
    const actionClient = new ROSLIB.ActionClient({
      ros: ros,
      serverName: '/panda_hand_controller/gripper_cmd', // Replace with your action server name
      actionName: 'control_msgs/GripperCommandAction ' // Replace with your action name
    });

    const goal = new ROSLIB.Goal({
      actionClient: actionClient,
      goalMessage: {
        command: {
          position: 0.00,
          max_effort: 1.0
        }
      }
    });

    goal.on('feedback', function (feedback) {
      console.log('Feedback:', feedback);
    });

    goal.on('result', function (result) {
      console.log('Final Result:', result);
    });

    console.log('Sending close gripper goal...');
    goal.send();

    goal.on('result', function (result) {
      console.log('Close gripper action completed with result:', result);
    });
  }



export function gripper(value){
    switch (value) {
        case 1:
        
        openGripper();
        break;
        case 2:
        
        closeGripper();
        break  

    }

    }

    




 

