import * as SpaceDriver from '../dist/space-driver.js';
import { ThreeDMouse } from '../dist/space-driver.js';
import { encode } from './jpeg-js/lib/encoder.js';

// Configure the callback

export var ros=new ROSLIB.Ros({
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
                x: -report.Ty_f,
                y: report.Tx_f,
                z: report.Tz_f
            },
            angular: {
                x: report.Ry_f,
                y: -report.Rx_f,
                z: report.Rz_f
            }
        }
    });
    var header = new ROSLIB.Message({
        stamp: {
            sec: Math.floor(Date.now() / 1000), // Current time in seconds
            nanosec: (Date.now() % 1000) * 1e6 // Current time in nanoseconds
        },
        frame_id: 'wrist_3_link' // Replace with your desired frame_id
    });

    // Assign the header to the TwistStamped message
    twist.header = header;

    

    // Publish the Twist message to the topic
    move_robotTopic.publish(twist);
    

    console.log("Published Twist message to robotTopic.");
}
const openTopic=new ROSLIB.Topic({
  ros:ros,
  name:'/open_grip',
  messageType:'std_msgs/Empty'
});

function openGripper() {
  console.log("Open gripper")
  const message=new ROSLIB.Message({
  });
  openTopic.publish(message)
    
}

const closeTopic=new ROSLIB.Topic({
  ros:ros,
  name:'/close_grip',
  messageType:'std_msgs/Empty'
});

function closeGripper() {
  console.log("Open gripper")
  const message=new ROSLIB.Message({
  });
  closeTopic.publish(message)
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
    
// ROSLIB image topic subscription
// const imageTopic = new ROSLIB.Topic({
//     ros: ros,
//     name: '/camera_sensor/image_raw',
//     messageType: 'sensor_msgs/msg/Image'
// });

// // Subscribe to the topic
// imageTopic.subscribe(function(message) {
//     // Process the message with your function
//     const base64Jpeg = rgb8ImageToBase64Jpeg(message);
    

//     if (base64Jpeg) {
//         // Display the image on your web interface
//         // Assuming you have an img tag with id="displayedImage"
//         const imageElement = document.getElementById("displayedImage");
//         imageElement.width =800;  // Desired width in pixels
//         imageElement.height = 500; // Desired height in pixels
//         console.log(base64Jpeg.substring(0, 100)); // log the first 100 characters


//         imageElement.src = "data:image/jpeg;base64," + base64Jpeg;
//     }
// });
// Usage
subscribeToCameraTopic(ros, '/camera/image_raw', 'displayedImageCamera0');
subscribeToCameraTopic(ros, '/camera1/image_raw', 'displayedImageCamera1');
subscribeToCameraTopic(ros, '/camera2/image_raw', 'displayedImageCamera2');


function subscribeToCameraTopic(ros, topicName, imageElementId) {
    const imageTopic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: 'sensor_msgs/msg/Image'
    });

    imageTopic.subscribe(function(message) {
        const base64Jpeg = rgb8ImageToBase64Jpeg(message);
        
        if (base64Jpeg) {
            const imageElement = document.getElementById(imageElementId);
            imageElement.width = 800;
            imageElement.height = 500;
            console.log(base64Jpeg.substring(0, 100));
            imageElement.src = "data:image/jpeg;base64," + base64Jpeg;
        }
    });
}

    
function rgb8ImageToBase64Jpeg(msg) {
    try {
        const raw = atob(msg.data);
        const array = new Uint8Array(new ArrayBuffer(raw.length));

        for (let i = 0; i < raw.length; i++) {
            array[i] = raw.charCodeAt(i);
        }

        const frameData = new Uint8Array(msg.width * msg.height * 4);

        for (let i = 0; i < msg.width * msg.height; i++) {
            frameData[4 * i + 0] = array[3 * i + 0];
            frameData[4 * i + 1] = array[3 * i + 1];
            frameData[4 * i + 2] = array[3 * i + 2];
            frameData[4 * i + 3] = 255;  // Set alpha value
        }

        const rawImageData = {
            data: frameData,
            width: msg.width,
            height: msg.height
        };

        // Convert to JPEG byte array
        const byteArray = encode(rawImageData, 50).data;
        
        // Convert the byte array to a Base64-encoded string
        const base64Jpeg = btoa(String.fromCharCode.apply(null, byteArray));

        return base64Jpeg;
    } catch (error) {
        console.error("Error in rgb8ImageToBase64Jpeg:", error);
        return null;  
    }
}

   
    
 
      



 

