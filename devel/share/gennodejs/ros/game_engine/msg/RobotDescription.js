// Auto-generated. Do not edit!

// (in-package game_engine.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RobotDescription {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.teamId = null;
      this.tagId = null;
      this.x = null;
      this.y = null;
      this.height = null;
      this.angle = null;
      this.addr0 = null;
      this.addr1 = null;
      this.vel1 = null;
      this.vel2 = null;
      this.previous_vel = null;
      this.collisionFlag = null;
      this.threadIsRunning = null;
      this.collisionStateVar = null;
      this.autonomous_drive = null;
      this.immobilized = null;
      this.damage = null;
      this.kills = null;
    }
    else {
      if (initObj.hasOwnProperty('teamId')) {
        this.teamId = initObj.teamId
      }
      else {
        this.teamId = 0;
      }
      if (initObj.hasOwnProperty('tagId')) {
        this.tagId = initObj.tagId
      }
      else {
        this.tagId = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
      if (initObj.hasOwnProperty('addr0')) {
        this.addr0 = initObj.addr0
      }
      else {
        this.addr0 = 0;
      }
      if (initObj.hasOwnProperty('addr1')) {
        this.addr1 = initObj.addr1
      }
      else {
        this.addr1 = 0;
      }
      if (initObj.hasOwnProperty('vel1')) {
        this.vel1 = initObj.vel1
      }
      else {
        this.vel1 = 0;
      }
      if (initObj.hasOwnProperty('vel2')) {
        this.vel2 = initObj.vel2
      }
      else {
        this.vel2 = 0;
      }
      if (initObj.hasOwnProperty('previous_vel')) {
        this.previous_vel = initObj.previous_vel
      }
      else {
        this.previous_vel = 0;
      }
      if (initObj.hasOwnProperty('collisionFlag')) {
        this.collisionFlag = initObj.collisionFlag
      }
      else {
        this.collisionFlag = false;
      }
      if (initObj.hasOwnProperty('threadIsRunning')) {
        this.threadIsRunning = initObj.threadIsRunning
      }
      else {
        this.threadIsRunning = false;
      }
      if (initObj.hasOwnProperty('collisionStateVar')) {
        this.collisionStateVar = initObj.collisionStateVar
      }
      else {
        this.collisionStateVar = 0;
      }
      if (initObj.hasOwnProperty('autonomous_drive')) {
        this.autonomous_drive = initObj.autonomous_drive
      }
      else {
        this.autonomous_drive = false;
      }
      if (initObj.hasOwnProperty('immobilized')) {
        this.immobilized = initObj.immobilized
      }
      else {
        this.immobilized = false;
      }
      if (initObj.hasOwnProperty('damage')) {
        this.damage = initObj.damage
      }
      else {
        this.damage = 0;
      }
      if (initObj.hasOwnProperty('kills')) {
        this.kills = initObj.kills
      }
      else {
        this.kills = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotDescription
    // Serialize message field [teamId]
    bufferOffset = _serializer.int8(obj.teamId, buffer, bufferOffset);
    // Serialize message field [tagId]
    bufferOffset = _serializer.int8(obj.tagId, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.int16(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.int16(obj.y, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float64(obj.angle, buffer, bufferOffset);
    // Serialize message field [addr0]
    bufferOffset = _serializer.uint8(obj.addr0, buffer, bufferOffset);
    // Serialize message field [addr1]
    bufferOffset = _serializer.uint8(obj.addr1, buffer, bufferOffset);
    // Serialize message field [vel1]
    bufferOffset = _serializer.int8(obj.vel1, buffer, bufferOffset);
    // Serialize message field [vel2]
    bufferOffset = _serializer.int8(obj.vel2, buffer, bufferOffset);
    // Serialize message field [previous_vel]
    bufferOffset = _serializer.int8(obj.previous_vel, buffer, bufferOffset);
    // Serialize message field [collisionFlag]
    bufferOffset = _serializer.bool(obj.collisionFlag, buffer, bufferOffset);
    // Serialize message field [threadIsRunning]
    bufferOffset = _serializer.bool(obj.threadIsRunning, buffer, bufferOffset);
    // Serialize message field [collisionStateVar]
    bufferOffset = _serializer.int8(obj.collisionStateVar, buffer, bufferOffset);
    // Serialize message field [autonomous_drive]
    bufferOffset = _serializer.bool(obj.autonomous_drive, buffer, bufferOffset);
    // Serialize message field [immobilized]
    bufferOffset = _serializer.bool(obj.immobilized, buffer, bufferOffset);
    // Serialize message field [damage]
    bufferOffset = _serializer.int8(obj.damage, buffer, bufferOffset);
    // Serialize message field [kills]
    bufferOffset = _serializer.int8(obj.kills, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotDescription
    let len;
    let data = new RobotDescription(null);
    // Deserialize message field [teamId]
    data.teamId = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [tagId]
    data.tagId = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [addr0]
    data.addr0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [addr1]
    data.addr1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vel1]
    data.vel1 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [vel2]
    data.vel2 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [previous_vel]
    data.previous_vel = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [collisionFlag]
    data.collisionFlag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [threadIsRunning]
    data.threadIsRunning = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [collisionStateVar]
    data.collisionStateVar = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [autonomous_drive]
    data.autonomous_drive = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [immobilized]
    data.immobilized = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [damage]
    data.damage = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [kills]
    data.kills = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 34;
  }

  static datatype() {
    // Returns string type for a message object
    return 'game_engine/RobotDescription';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '35b35bffdcea4af3c10704857437163d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new RobotDescription(null);
    if (msg.teamId !== undefined) {
      resolved.teamId = msg.teamId;
    }
    else {
      resolved.teamId = 0
    }

    if (msg.tagId !== undefined) {
      resolved.tagId = msg.tagId;
    }
    else {
      resolved.tagId = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    if (msg.addr0 !== undefined) {
      resolved.addr0 = msg.addr0;
    }
    else {
      resolved.addr0 = 0
    }

    if (msg.addr1 !== undefined) {
      resolved.addr1 = msg.addr1;
    }
    else {
      resolved.addr1 = 0
    }

    if (msg.vel1 !== undefined) {
      resolved.vel1 = msg.vel1;
    }
    else {
      resolved.vel1 = 0
    }

    if (msg.vel2 !== undefined) {
      resolved.vel2 = msg.vel2;
    }
    else {
      resolved.vel2 = 0
    }

    if (msg.previous_vel !== undefined) {
      resolved.previous_vel = msg.previous_vel;
    }
    else {
      resolved.previous_vel = 0
    }

    if (msg.collisionFlag !== undefined) {
      resolved.collisionFlag = msg.collisionFlag;
    }
    else {
      resolved.collisionFlag = false
    }

    if (msg.threadIsRunning !== undefined) {
      resolved.threadIsRunning = msg.threadIsRunning;
    }
    else {
      resolved.threadIsRunning = false
    }

    if (msg.collisionStateVar !== undefined) {
      resolved.collisionStateVar = msg.collisionStateVar;
    }
    else {
      resolved.collisionStateVar = 0
    }

    if (msg.autonomous_drive !== undefined) {
      resolved.autonomous_drive = msg.autonomous_drive;
    }
    else {
      resolved.autonomous_drive = false
    }

    if (msg.immobilized !== undefined) {
      resolved.immobilized = msg.immobilized;
    }
    else {
      resolved.immobilized = false
    }

    if (msg.damage !== undefined) {
      resolved.damage = msg.damage;
    }
    else {
      resolved.damage = 0
    }

    if (msg.kills !== undefined) {
      resolved.kills = msg.kills;
    }
    else {
      resolved.kills = 0
    }

    return resolved;
    }
};

module.exports = RobotDescription;
