import {ensureVector3} from "../dist/linAlg.js";

export class Robot {
  constructor(ros) {
    this.ros = ros
    this.twistTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/servo_node/delta_twist_cmds',
      messageType: 'geometry_msgs/msg/TwistStamped'
    });
  }

  move(twist, frame="base_link") {
    let [linear, angular] = [ensureVector3(twist[0]), ensureVector3(twist[1])]
    let linearValues = {
      x: linear.x,
      y: linear.y,
      z: linear.z
    };

    let angularValues = {
      x: angular.x,
      y: angular.y,
      z: angular.z
    };

    const twistMsg = new ROSLIB.Message({
      twist: {
        linear: linearValues,
        angular: angularValues
      }
    });

    twistMsg.header = new ROSLIB.Message({
        stamp: {
            sec: Math.floor(Date.now() / 1000),
            nanosec: (Date.now() % 1000) * 1e6
        },
        frame_id: frame
    });
    this.twistTopic.publish(twistMsg);
  }

  openGripper() {
    console.log('Open gripper');
    const message = new ROSLIB.Message({});
    this.openTopic.publish(message);
  }

  closeGripper() {
    console.log('Close gripper');
    const message = new ROSLIB.Message({});
    this.closeTopic.publish(message);
  }

  controlGripper(value) {
    if (value === 1) {
      this.openGripper();
    } else if (value === 2) {
      this.closeGripper();
    }
  }
}


export function subscribeToCameraTopic(ros, topicName, element) {
    const imageTopic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: 'sensor_msgs/msg/Image'
    });
    let ctx = element.getContext('2d');
    imageTopic.subscribe(function(message) {
        element.width = message.width;
        element.height = message.height;
        let imageData = ctx.createImageData(message.width, message.height);
        imageData = rgb8ImageToImageData(message, imageData)
        // Iterate through every pixel
        ctx.putImageData(imageData, 0, 0);
    
    });
}
    

function rgb8ImageToImageData(msg, outData) {
    const raw = atob(msg.data);
    const array = new Uint8Array(new ArrayBuffer(raw.length));

    for (let i = 0; i < raw.length; i++) {
        array[i] = raw.charCodeAt(i);
    }

    for (let i = 0; i < msg.width * msg.height; i++) {
        outData.data[4 * i + 0] = array[3 * i + 0];
        outData.data[4 * i + 1] = array[3 * i + 1];
        outData.data[4 * i + 2] = array[3 * i + 2];
        outData.data[4 * i + 3] = 255;  // Set alpha value
    }

    return outData;
}
