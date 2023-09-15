const ROSLIB = require('roslib');

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

// Replace 'your_action_server' with your action server name and 
// 'your_action_msg_type' with your action message type
const actionClient = new ROSLIB.ActionClient({
  ros: ros,
  serverName: '/panda_hand_controller/gripper_cmd',
  actionName: 'control_msgs/action/GripperCommand'
});

// Replace 'your_goal_object' with the goal object you want to send
const goal = new ROSLIB.Goal({
  actionClient: actionClient,
  goalMessage: {
    command: {
      position: 0.04,
      max_effort: 2.0
    }
  }
});

goal.on('feedback', function(feedback) {
  console.log('Feedback: ' + feedback);
});

goal.on('result', function(result) {
  console.log('Final Result: ' + result);
});

goal.send();

goal.on('result', function(result) {
  console.log('Action executed with result: ' + result);
});
