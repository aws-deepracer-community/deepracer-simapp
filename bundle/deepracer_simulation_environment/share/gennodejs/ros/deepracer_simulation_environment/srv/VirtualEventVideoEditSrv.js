// Auto-generated. Do not edit!

// (in-package deepracer_simulation_environment.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class VirtualEventVideoEditSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.display_name = null;
      this.racecar_color = null;
    }
    else {
      if (initObj.hasOwnProperty('display_name')) {
        this.display_name = initObj.display_name
      }
      else {
        this.display_name = '';
      }
      if (initObj.hasOwnProperty('racecar_color')) {
        this.racecar_color = initObj.racecar_color
      }
      else {
        this.racecar_color = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VirtualEventVideoEditSrvRequest
    // Serialize message field [display_name]
    bufferOffset = _serializer.string(obj.display_name, buffer, bufferOffset);
    // Serialize message field [racecar_color]
    bufferOffset = _serializer.string(obj.racecar_color, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VirtualEventVideoEditSrvRequest
    let len;
    let data = new VirtualEventVideoEditSrvRequest(null);
    // Deserialize message field [display_name]
    data.display_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [racecar_color]
    data.racecar_color = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.display_name.length;
    length += object.racecar_color.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation_environment/VirtualEventVideoEditSrvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a20c4716b8ea42a4d682ce4ccd9435e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string display_name
    string racecar_color
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VirtualEventVideoEditSrvRequest(null);
    if (msg.display_name !== undefined) {
      resolved.display_name = msg.display_name;
    }
    else {
      resolved.display_name = ''
    }

    if (msg.racecar_color !== undefined) {
      resolved.racecar_color = msg.racecar_color;
    }
    else {
      resolved.racecar_color = ''
    }

    return resolved;
    }
};

class VirtualEventVideoEditSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VirtualEventVideoEditSrvResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VirtualEventVideoEditSrvResponse
    let len;
    let data = new VirtualEventVideoEditSrvResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation_environment/VirtualEventVideoEditSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VirtualEventVideoEditSrvResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: VirtualEventVideoEditSrvRequest,
  Response: VirtualEventVideoEditSrvResponse,
  md5sum() { return 'd375256530e7fab3e8486078c126cdb6'; },
  datatype() { return 'deepracer_simulation_environment/VirtualEventVideoEditSrv'; }
};
