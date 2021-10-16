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

class StateRotaryServo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.goal = null;
      this.position = null;
      this.error = null;
      this.velocity = null;
      this.effort = null;
      this.load = null;
      this.moving = null;
      this.fault = null;
      this.control_type = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('effort')) {
        this.effort = initObj.effort
      }
      else {
        this.effort = 0.0;
      }
      if (initObj.hasOwnProperty('load')) {
        this.load = initObj.load
      }
      else {
        this.load = 0.0;
      }
      if (initObj.hasOwnProperty('moving')) {
        this.moving = initObj.moving
      }
      else {
        this.moving = false;
      }
      if (initObj.hasOwnProperty('fault')) {
        this.fault = initObj.fault
      }
      else {
        this.fault = 0;
      }
      if (initObj.hasOwnProperty('control_type')) {
        this.control_type = initObj.control_type
      }
      else {
        this.control_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StateRotaryServo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [goal]
    bufferOffset = _serializer.float64(obj.goal, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float64(obj.position, buffer, bufferOffset);
    // Serialize message field [error]
    bufferOffset = _serializer.float64(obj.error, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    // Serialize message field [effort]
    bufferOffset = _serializer.float64(obj.effort, buffer, bufferOffset);
    // Serialize message field [load]
    bufferOffset = _serializer.float64(obj.load, buffer, bufferOffset);
    // Serialize message field [moving]
    bufferOffset = _serializer.bool(obj.moving, buffer, bufferOffset);
    // Serialize message field [fault]
    bufferOffset = _serializer.uint8(obj.fault, buffer, bufferOffset);
    // Serialize message field [control_type]
    bufferOffset = _serializer.uint8(obj.control_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StateRotaryServo
    let len;
    let data = new StateRotaryServo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error]
    data.error = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [effort]
    data.effort = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [load]
    data.load = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [moving]
    data.moving = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fault]
    data.fault = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [control_type]
    data.control_type = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 51;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lsbot_msgs/StateRotaryServo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7b87918793ebff92fa52e8d24df3ee5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # motor condition and the reason in case of error
    
    std_msgs/Header header # motor state is at this time
    
    float64 goal # commanded position
    
    float64 position # current position encoder
    
    float64 error # difference between current and goal positions
    
    float64 velocity # current velocity
    
    float64 effort # current effort of the actuator
    
    float64 load # load imposed on the motor
    
    bool moving # whether the motor is currently in motion
    
    uint8 FAULT_NONE=0
    uint8 FAULT_CURRENT=1
    uint8 FAULT_GENERAL=2
    uint8 fault # fault cause
    
    uint8 CONTROL_TYPE_NONE=0
    uint8 CONTROL_TYPE_POSITION=1
    uint8 CONTROL_TYPE_EFFORT=2
    uint8 CONTROL_TYPE_VELOCITY=3
    uint8 CONTROL_TYPE_POSITION_VELOCITY=4
    uint8 CONTROL_TYPE_POSITION_EFFORT=5
    uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
    uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
    uint8 control_type # rotary servomotor control type
    
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
    const resolved = new StateRotaryServo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.goal !== undefined) {
      resolved.goal = msg.goal;
    }
    else {
      resolved.goal = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.effort !== undefined) {
      resolved.effort = msg.effort;
    }
    else {
      resolved.effort = 0.0
    }

    if (msg.load !== undefined) {
      resolved.load = msg.load;
    }
    else {
      resolved.load = 0.0
    }

    if (msg.moving !== undefined) {
      resolved.moving = msg.moving;
    }
    else {
      resolved.moving = false
    }

    if (msg.fault !== undefined) {
      resolved.fault = msg.fault;
    }
    else {
      resolved.fault = 0
    }

    if (msg.control_type !== undefined) {
      resolved.control_type = msg.control_type;
    }
    else {
      resolved.control_type = 0
    }

    return resolved;
    }
};

// Constants for message
StateRotaryServo.Constants = {
  FAULT_NONE: 0,
  FAULT_CURRENT: 1,
  FAULT_GENERAL: 2,
  CONTROL_TYPE_NONE: 0,
  CONTROL_TYPE_POSITION: 1,
  CONTROL_TYPE_EFFORT: 2,
  CONTROL_TYPE_VELOCITY: 3,
  CONTROL_TYPE_POSITION_VELOCITY: 4,
  CONTROL_TYPE_POSITION_EFFORT: 5,
  CONTROL_TYPE_VELOCITY_EFFORT: 6,
  CONTROL_TYPE_POSITION_VELOCITY_EFFORT: 7,
}

module.exports = StateRotaryServo;
