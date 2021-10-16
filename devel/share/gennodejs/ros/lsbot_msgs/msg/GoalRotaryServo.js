// Auto-generated. Do not edit!

// (in-package lsbot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GoalRotaryServo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.control_type = null;
      this.position = null;
      this.velocity = null;
      this.acceleration = null;
      this.effort = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('control_type')) {
        this.control_type = initObj.control_type
      }
      else {
        this.control_type = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = 0.0;
      }
      if (initObj.hasOwnProperty('effort')) {
        this.effort = initObj.effort
      }
      else {
        this.effort = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GoalRotaryServo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [control_type]
    bufferOffset = _serializer.uint8(obj.control_type, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float64(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float32(obj.velocity, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = _serializer.float32(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [effort]
    bufferOffset = _serializer.float32(obj.effort, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GoalRotaryServo
    let len;
    let data = new GoalRotaryServo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [control_type]
    data.control_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [effort]
    data.effort = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lsbot_msgs/GoalRotaryServo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd7c284381d46455f889dcdafb56f5389';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # control the position, velocity or/and effort
    
    std_msgs/Header header
    
    uint8 CONTROL_TYPE_NONE=0
    uint8 CONTROL_TYPE_POSITION=1
    uint8 CONTROL_TYPE_EFFORT=2
    uint8 CONTROL_TYPE_VELOCITY=3
    uint8 CONTROL_TYPE_POSITION_VELOCITY=4
    uint8 CONTROL_TYPE_POSITION_EFFORT=5
    uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
    uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
    uint8 control_type # rotary servomotor control type
    
    float64 position # goal position
    
    float32 velocity # movement velocity
    
    float32 acceleration # movement acceleration
    
    float32 effort # movement torque
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GoalRotaryServo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.control_type !== undefined) {
      resolved.control_type = msg.control_type;
    }
    else {
      resolved.control_type = 0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = msg.acceleration;
    }
    else {
      resolved.acceleration = 0.0
    }

    if (msg.effort !== undefined) {
      resolved.effort = msg.effort;
    }
    else {
      resolved.effort = 0.0
    }

    return resolved;
    }
};

// Constants for message
GoalRotaryServo.Constants = {
  CONTROL_TYPE_NONE: 0,
  CONTROL_TYPE_POSITION: 1,
  CONTROL_TYPE_EFFORT: 2,
  CONTROL_TYPE_VELOCITY: 3,
  CONTROL_TYPE_POSITION_VELOCITY: 4,
  CONTROL_TYPE_POSITION_EFFORT: 5,
  CONTROL_TYPE_VELOCITY_EFFORT: 6,
  CONTROL_TYPE_POSITION_VELOCITY_EFFORT: 7,
}

module.exports = GoalRotaryServo;
