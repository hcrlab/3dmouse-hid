import { ensureVector3 } from "../dist/linAlg.js";

/**
 * A class representing a robot connecting via ROS
 */

export class Robot {
  /**
   * Create a Robot instance.
   * @param {object} ros - The ROSLIB Ros instance to connect to.
   */

  constructor(ros, options = {}) {
    options = {...{twistTopicName: "/threedmouse/twist", buttonTopicName: "/threedmouse/buttons", buttonEventTopicName: "/threedmouse/button_event"}, ...options};
    this.ros = ros;
    this.twistTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: options.twistTopicName,
      messageType: "geometry_msgs/msg/TwistStamped",
    });

    // JSON string of button state dictionary
    this.buttonsTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: options.buttonTopicName,
      messageType: "std_msgs/msg/String",
    });

    // We have avoided defining any ROS message types because
    // we don't want the main package to depend on ROS, and because
    // we don't want to require building some other package just to use the demo.
    // So we'll string-ly type the button up/down events
    this.buttonChangeTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: options.buttonEventTopicName,
      messageType: "std_msgs/msg/String",
    });
  }

  /**
   * Send a movement command to the robot.
   * @param {array} twist - An array with linear and angular movement vectors.
   * @param {string} [frame="base_link"] - The reference frame for the movement.
   * @param {number} [time=null] - The timestamp for the command. Uses current time if not provided.
   */

  move(twist, frame = "base_link", time = null) {
    let [linear, angular] = [ensureVector3(twist[0]), ensureVector3(twist[1])];
    let linearValues = {
      x: linear.x,
      y: linear.y,
      z: linear.z,
    };

    let angularValues = {
      x: angular.x,
      y: angular.y,
      z: angular.z,
    };

    const twistMsg = new ROSLIB.Message({
      twist: {
        linear: linearValues,
        angular: angularValues,
      },
    });

    if (!time) {
      time = Date.now();
    }
    twistMsg.header = new ROSLIB.Message({
      stamp: {
        sec: Math.floor(time / 1000),
        nanosec: (time % 1000) * 1e6,
      },
      frame_id: frame,
    });
    this.twistTopic.publish(twistMsg);
  }

  forwardButtonStates(states) {
    this.buttonsTopic.publish(new ROSLIB.Message({ data: JSON.stringify(states) }));
  }
  forwardButtonChange(name, state) {
    this.buttonChangeTopic.publish(new ROSLIB.Message({ data: `${name},${state}`}));
  }

}

/**
 * Initialize and connect to the ROS system.
 * @returns {Promise} Resolves with the ROSLIB Ros instance if successful, rejects with an error otherwise.
 */

export function initializeRos(host="ws://127.0.0.1:9090") {
  return new Promise((resolve, reject) => {
    let ros = new ROSLIB.Ros({
      url: host,
    });

    ros.on("connection", () => {
      resolve(ros);
    });

    ros.on("error", (error) => {
      reject(error);
    });
  });
}

/**
 * Subscribe to a camera topic in ROS and render the images on a canvas element.
 * @param {object} ros - The ROSLIB Ros instance to connect to.
 * @param {string} topicName - The name of the camera topic in ROS.
 * @param {HTMLCanvasElement} element - The canvas element to render the images on.
 * @param {boolean} [compressed=null] - Whether the camera topic is compressed. If not provided, the function will attempt to determine this from the topic name.
 */

export function subscribeToCameraTopic(ros, topicName, element, compressed=null) {
  if (compressed === null) {
    compressed = topicName.endsWith("compressed");
  }

  if (compressed) {
    const imageTopic = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: "sensor_msgs/msg/CompressedImage",
    });
    let ctx = element.getContext("2d");
    imageTopic.subscribe(function (message) {
      let img = new Image();
      img.onload = function () {
        element.width = img.width;
        element.height = img.height;
        ctx.drawImage(img, 0, 0);
      };
      img.src = "data:image/jpeg;base64," + message.data;
    });
    return;
  } else {
    const imageTopic = new ROSLIB.Topic({
      ros: ros,
      name: topicName,
      messageType: "sensor_msgs/msg/Image",
    });
    let ctx = element.getContext("2d");
    imageTopic.subscribe(function (message) {
      element.width = message.width;
      element.height = message.height;
      let imageData = ctx.createImageData(message.width, message.height);
      imageData = rgb8ImageToImageData(message, imageData);
      // Iterate through every pixel
      ctx.putImageData(imageData, 0, 0);
    });
  }
}

/**
 * Convert an RGB8 image message from ROS to an ImageData object.
 * @param {object} msg - The ROS image message.
 * @param {ImageData} outData - An ImageData object to write the image data to.
 * @returns {ImageData} The resulting ImageData object.
 */

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
    outData.data[4 * i + 3] = 255; // Set alpha value
  }

  return outData;
}
