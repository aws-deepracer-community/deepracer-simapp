// Auto-generated. Do not edit!

// (in-package deepracer_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetLightNamesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetLightNamesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetLightNamesRequest
    let len;
    let data = new GetLightNamesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetLightNamesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetLightNamesRequest(null);
    return resolved;
    }
};

class GetLightNamesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.light_names = null;
      this.success = null;
      this.status_message = null;
    }
    else {
      if (initObj.hasOwnProperty('light_names')) {
        this.light_names = initObj.light_names
      }
      else {
        this.light_names = [];
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('status_message')) {
        this.status_message = initObj.status_message
      }
      else {
        this.status_message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetLightNamesResponse
    // Serialize message field [light_names]
    bufferOffset = _arraySerializer.string(obj.light_names, buffer, bufferOffset, null);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [status_message]
    bufferOffset = _serializer.string(obj.status_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetLightNamesResponse
    let len;
    let data = new GetLightNamesResponse(null);
    // Deserialize message field [light_names]
    data.light_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status_message]
    data.status_message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.light_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += object.status_message.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetLightNamesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6dff1e71fe0fb6f3d275724e4c746d5e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] light_names
    bool success
    string status_message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetLightNamesResponse(null);
    if (msg.light_names !== undefined) {
      resolved.light_names = msg.light_names;
    }
    else {
      resolved.light_names = []
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.status_message !== undefined) {
      resolved.status_message = msg.status_message;
    }
    else {
      resolved.status_message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GetLightNamesRequest,
  Response: GetLightNamesResponse,
  md5sum() { return '6dff1e71fe0fb6f3d275724e4c746d5e'; },
  datatype() { return 'deepracer_msgs/GetLightNames'; }
};
