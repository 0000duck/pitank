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

class UIState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.setupStart = null;
      this.gameStart = null;
      this.teamGame = null;
      this.paused = null;
      this.aiGame = null;
      this.seconds = null;
    }
    else {
      if (initObj.hasOwnProperty('setupStart')) {
        this.setupStart = initObj.setupStart
      }
      else {
        this.setupStart = false;
      }
      if (initObj.hasOwnProperty('gameStart')) {
        this.gameStart = initObj.gameStart
      }
      else {
        this.gameStart = false;
      }
      if (initObj.hasOwnProperty('teamGame')) {
        this.teamGame = initObj.teamGame
      }
      else {
        this.teamGame = false;
      }
      if (initObj.hasOwnProperty('paused')) {
        this.paused = initObj.paused
      }
      else {
        this.paused = false;
      }
      if (initObj.hasOwnProperty('aiGame')) {
        this.aiGame = initObj.aiGame
      }
      else {
        this.aiGame = false;
      }
      if (initObj.hasOwnProperty('seconds')) {
        this.seconds = initObj.seconds
      }
      else {
        this.seconds = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UIState
    // Serialize message field [setupStart]
    bufferOffset = _serializer.bool(obj.setupStart, buffer, bufferOffset);
    // Serialize message field [gameStart]
    bufferOffset = _serializer.bool(obj.gameStart, buffer, bufferOffset);
    // Serialize message field [teamGame]
    bufferOffset = _serializer.bool(obj.teamGame, buffer, bufferOffset);
    // Serialize message field [paused]
    bufferOffset = _serializer.bool(obj.paused, buffer, bufferOffset);
    // Serialize message field [aiGame]
    bufferOffset = _serializer.bool(obj.aiGame, buffer, bufferOffset);
    // Serialize message field [seconds]
    bufferOffset = _serializer.int32(obj.seconds, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UIState
    let len;
    let data = new UIState(null);
    // Deserialize message field [setupStart]
    data.setupStart = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gameStart]
    data.gameStart = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [teamGame]
    data.teamGame = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [paused]
    data.paused = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [aiGame]
    data.aiGame = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [seconds]
    data.seconds = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'game_engine/UIState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3e166d509eed64c7b231607645e715e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool setupStart
    bool gameStart
    bool teamGame
    bool paused
    bool aiGame
    int32 seconds
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UIState(null);
    if (msg.setupStart !== undefined) {
      resolved.setupStart = msg.setupStart;
    }
    else {
      resolved.setupStart = false
    }

    if (msg.gameStart !== undefined) {
      resolved.gameStart = msg.gameStart;
    }
    else {
      resolved.gameStart = false
    }

    if (msg.teamGame !== undefined) {
      resolved.teamGame = msg.teamGame;
    }
    else {
      resolved.teamGame = false
    }

    if (msg.paused !== undefined) {
      resolved.paused = msg.paused;
    }
    else {
      resolved.paused = false
    }

    if (msg.aiGame !== undefined) {
      resolved.aiGame = msg.aiGame;
    }
    else {
      resolved.aiGame = false
    }

    if (msg.seconds !== undefined) {
      resolved.seconds = msg.seconds;
    }
    else {
      resolved.seconds = 0
    }

    return resolved;
    }
};

module.exports = UIState;
