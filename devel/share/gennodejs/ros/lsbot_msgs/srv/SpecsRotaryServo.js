// Auto-generated. Do not edit!

// (in-package lsbot_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SpecsRotaryServoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SpecsRotaryServoRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SpecsRotaryServoRequest
    let len;
    let data = new SpecsRotaryServoRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lsbot_msgs/SpecsRotaryServoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # device features
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SpecsRotaryServoRequest(null);
    return resolved;
    }
};

class SpecsRotaryServoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_type = null;
      this.range_min = null;
      this.range_max = null;
      this.precision = null;
      this.rated_speed = null;
      this.reachable_speed = null;
      this.rated_torque = null;
      this.reachable_torque = null;
      this.temperature_range_min = null;
      this.temperature_range_max = null;
    }
    else {
      if (initObj.hasOwnProperty('control_type')) {
        this.control_type = initObj.control_type
      }
      else {
        this.control_type = 0;
      }
      if (initObj.hasOwnProperty('range_min')) {
        this.range_min = initObj.range_min
      }
      else {
        this.range_min = 0.0;
      }
      if (initObj.hasOwnProperty('range_max')) {
        this.range_max = initObj.range_max
      }
      else {
        this.range_max = 0.0;
      }
      if (initObj.hasOwnProperty('precision')) {
        this.precision = initObj.precision
      }
      else {
        this.precision = 0.0;
      }
      if (initObj.hasOwnProperty('rated_speed')) {
        this.rated_speed = initObj.rated_speed
      }
      else {
        this.rated_speed = 0.0;
      }
      if (initObj.hasOwnProperty('reachable_speed')) {
        this.reachable_speed = initObj.reachable_speed
      }
      else {
        this.reachable_speed = 0.0;
      }
      if (initObj.hasOwnProperty('rated_torque')) {
        this.rated_torque = initObj.rated_torque
      }
      else {
        this.rated_torque = 0.0;
      }
      if (initObj.hasOwnProperty('reachable_torque')) {
        this.reachable_torque = initObj.reachable_torque
      }
      else {
        this.reachable_torque = 0.0;
      }
      if (initObj.hasOwnProperty('temperature_range_min')) {
        this.temperature_range_min = initObj.temperature_range_min
      }
      else {
        this.temperature_range_min = 0.0;
      }
      if (initObj.hasOwnProperty('temperature_range_max')) {
        this.temperature_range_max = initObj.temperature_range_max
      }
      else {
        this.temperature_range_max = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SpecsRotaryServoResponse
    // Serialize message field [control_type]
    bufferOffset = _serializer.uint8(obj.control_type, buffer, bufferOffset);
    // Serialize message field [range_min]
    bufferOffset = _serializer.float64(obj.range_min, buffer, bufferOffset);
    // Serialize message field [range_max]
    bufferOffset = _serializer.float64(obj.range_max, buffer, bufferOffset);
    // Serialize message field [precision]
    bufferOffset = _serializer.float64(obj.precision, buffer, bufferOffset);
    // Serialize message field [rated_speed]
    bufferOffset = _serializer.float64(obj.rated_speed, buffer, bufferOffset);
    // Serialize message field [reachable_speed]
    bufferOffset = _serializer.float64(obj.reachable_speed, buffer, bufferOffset);
    // Serialize message field [rated_torque]
    bufferOffset = _serializer.float64(obj.rated_torque, buffer, bufferOffset);
    // Serialize message field [reachable_torque]
    bufferOffset = _serializer.float64(obj.reachable_torque, buffer, bufferOffset);
    // Serialize message field [temperature_range_min]
    bufferOffset = _serializer.float64(obj.temperature_range_min, buffer, bufferOffset);
    // Serialize message field [temperature_range_max]
    bufferOffset = _serializer.float64(obj.temperature_range_max, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SpecsRotaryServoResponse
    let len;
    let data = new SpecsRotaryServoResponse(null);
    // Deserialize message field [control_type]
    data.control_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [range_min]
    data.range_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [range_max]
    data.range_max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [precision]
    data.precision = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rated_speed]
    data.rated_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [reachable_speed]
    data.reachable_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rated_torque]
    data.rated_torque = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [reachable_torque]
    data.reachable_torque = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temperature_range_min]
    data.temperature_range_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temperature_range_max]
    data.temperature_range_max = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 73;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lsbot_msgs/SpecsRotaryServoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c99b39095d63ff4cbaed9c5a6eec7d20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 CONTROL_TYPE_NONE=0
    uint8 CONTROL_TYPE_POSITION=1
    uint8 CONTROL_TYPE_EFFORT=2
    uint8 CONTROL_TYPE_VELOCITY=3
    uint8 CONTROL_TYPE_POSITION_VELOCITY=4
    uint8 CONTROL_TYPE_POSITION_EFFORT=5
    uint8 CONTROL_TYPE_VELOCITY_EFFORT=6
    uint8 CONTROL_TYPE_POSITION_VELOCITY_EFFORT=7
    uint8 control_type # rotary servomotor control type
    
    float64 range_min # minimum work range
    
    float64 range_max # maximum work range
    
    float64 precision # angular precision
    
    float64 rated_speed # servomotor speed
    
    float64 reachable_speed # maximum speed
    
    float64 rated_torque # servomotor torque
    
    float64 reachable_torque # peak torque
    
    float64 temperature_range_min # minimum operational temperature
    
    float64 temperature_range_max # maximum operational temperature
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SpecsRotaryServoResponse(null);
    if (msg.control_type !== undefined) {
      resolved.control_type = msg.control_type;
    }
    else {
      resolved.control_type = 0
    }

    if (msg.range_min !== undefined) {
      resolved.range_min = msg.range_min;
    }
    else {
      resolved.range_min = 0.0
    }

    if (msg.range_max !== undefined) {
      resolved.range_max = msg.range_max;
    }
    else {
      resolved.range_max = 0.0
    }

    if (msg.precision !== undefined) {
      resolved.precision = msg.precision;
    }
    else {
      resolved.precision = 0.0
    }

    if (msg.rated_speed !== undefined) {
      resolved.rated_speed = msg.rated_speed;
    }
    else {
      resolved.rated_speed = 0.0
    }

    if (msg.reachable_speed !== undefined) {
      resolved.reachable_speed = msg.reachable_speed;
    }
    else {
      resolved.reachable_speed = 0.0
    }

    if (msg.rated_torque !== undefined) {
      resolved.rated_torque = msg.rated_torque;
    }
    else {
      resolved.rated_torque = 0.0
    }

    if (msg.reachable_torque !== undefined) {
      resolved.reachable_torque = msg.reachable_torque;
    }
    else {
      resolved.reachable_torque = 0.0
    }

    if (msg.temperature_range_min !== undefined) {
      resolved.temperature_range_min = msg.temperature_range_min;
    }
    else {
      resolved.temperature_range_min = 0.0
    }

    if (msg.temperature_range_max !== undefined) {
      resolved.temperature_range_max = msg.temperature_range_max;
    }
    else {
      resolved.temperature_range_max = 0.0
    }

    return resolved;
    }
};

// Constants for message
SpecsRotaryServoResponse.Constants = {
  CONTROL_TYPE_NONE: 0,
  CONTROL_TYPE_POSITION: 1,
  CONTROL_TYPE_EFFORT: 2,
  CONTROL_TYPE_VELOCITY: 3,
  CONTROL_TYPE_POSITION_VELOCITY: 4,
  CONTROL_TYPE_POSITION_EFFORT: 5,
  CONTROL_TYPE_VELOCITY_EFFORT: 6,
  CONTROL_TYPE_POSITION_VELOCITY_EFFORT: 7,
}

module.exports = {
  Request: SpecsRotaryServoRequest,
  Response: SpecsRotaryServoResponse,
  md5sum() { return 'c99b39095d63ff4cbaed9c5a6eec7d20'; },
  datatype() { return 'lsbot_msgs/SpecsRotaryServo'; }
};
