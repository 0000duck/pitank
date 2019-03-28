// Auto-generated. Do not edit!

// (in-package game_engine.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RobotDescription = require('./RobotDescription.js');

//-----------------------------------------------------------

class RobotDescriptionArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot = null;
    }
    else {
      if (initObj.hasOwnProperty('robot')) {
        this.robot = initObj.robot
      }
      else {
        this.robot = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotDescriptionArray
    // Serialize message field [robot]
    // Serialize the length for message field [robot]
    bufferOffset = _serializer.uint32(obj.robot.length, buffer, bufferOffset);
    obj.robot.forEach((val) => {
      bufferOffset = RobotDescription.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotDescriptionArray
    let len;
    let data = new RobotDescriptionArray(null);
    // Deserialize message field [robot]
    // Deserialize array length for message field [robot]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.robot = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.robot[i] = RobotDescription.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 34 * object.robot.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'game_engine/RobotDescriptionArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a10b3056d03141a072c7ffcaaf2ce2ff';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    RobotDescription[] robot
    
    ================================================================================
    MSG: game_engine/RobotDescription
    int8 teamId
    int8 tagId
    int16 x
    int16 y
    float64 height
    float64 angle
    uint8 addr0
    uint8 addr1
    int8 vel1
    int8 vel2
    int8 previous_vel
    bool collisionFlag
    bool threadIsRunning
    int8 collisionStateVar
    bool autonomous_drive
    bool immobilized
    int8 damage
    int8 kills
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotDescriptionArray(null);
    if (msg.robot !== undefined) {
      resolved.robot = new Array(msg.robot.length);
      for (let i = 0; i < resolved.robot.length; ++i) {
        resolved.robot[i] = RobotDescription.Resolve(msg.robot[i]);
      }
    }
    else {
      resolved.robot = []
    }

    return resolved;
    }
};

module.exports = RobotDescriptionArray;
