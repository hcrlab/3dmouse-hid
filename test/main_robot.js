import * as SpaceDriver from '../dist/space-driver.js';
SpaceDriver.configureCallback(move_robot)

//Create a new Ros instance and connect to the Server

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

var robotTopic= new ROSLIB.Topic({
    ros : ros,
    name : '/servo_node/delta_twist_cmds',
    messagType : 'geometry_msgs/msg/Twist'
  });


function move_robot(){
    console.log("Teleop Twist");
    var twist=new ROSLIB.Message({
        linear: {
            x:Tx,
            y:Ty,
            z:Tz
        },
        angular:{
            x:0,
            y:0,
            z:angular
        }
    });
    robotTopic.publish(twist);



}
