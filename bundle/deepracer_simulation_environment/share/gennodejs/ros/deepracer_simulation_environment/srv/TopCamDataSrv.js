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

class TopCamDataSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TopCamDataSrvRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TopCamDataSrvRequest
    let len;
    let data = new TopCamDataSrvRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation_environment/TopCamDataSrvRequest';
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
    const resolved = new TopCamDataSrvRequest(null);
    return resolved;
    }
};

class TopCamDataSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.horizontal_fov = null;
      this.padding_pct = null;
      this.image_width = null;
      this.image_height = null;
    }
    else {
      if (initObj.hasOwnProperty('horizontal_fov')) {
        this.horizontal_fov = initObj.horizontal_fov
      }
      else {
        this.horizontal_fov = 0.0;
      }
      if (initObj.hasOwnProperty('padding_pct')) {
        this.padding_pct = initObj.padding_pct
      }
      else {
        this.padding_pct = 0.0;
      }
      if (initObj.hasOwnProperty('image_width')) {
        this.image_width = initObj.image_width
      }
      else {
        this.image_width = 0.0;
      }
      if (initObj.hasOwnProperty('image_height')) {
        this.image_height = initObj.image_height
      }
      else {
        this.image_height = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TopCamDataSrvResponse
    // Serialize message field [horizontal_fov]
    bufferOffset = _serializer.float32(obj.horizontal_fov, buffer, bufferOffset);
    // Serialize message field [padding_pct]
    bufferOffset = _serializer.float32(obj.padding_pct, buffer, bufferOffset);
    // Serialize message field [image_width]
    bufferOffset = _serializer.float32(obj.image_width, buffer, bufferOffset);
    // Serialize message field [image_height]
    bufferOffset = _serializer.float32(obj.image_height, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TopCamDataSrvResponse
    let len;
    let data = new TopCamDataSrvResponse(null);
    // Deserialize message field [horizontal_fov]
    data.horizontal_fov = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [padding_pct]
    data.padding_pct = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [image_width]
    data.image_width = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [image_height]
    data.image_height = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation_environment/TopCamDataSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eed9195ebff712c1941314f098f70944';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 horizontal_fov
    float32 padding_pct
    float32 image_width
    float32 image_height
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TopCamDataSrvResponse(null);
    if (msg.horizontal_fov !== undefined) {
      resolved.horizontal_fov = msg.horizontal_fov;
    }
    else {
      resolved.horizontal_fov = 0.0
    }

    if (msg.padding_pct !== undefined) {
      resolved.padding_pct = msg.padding_pct;
    }
    else {
      resolved.padding_pct = 0.0
    }

    if (msg.image_width !== undefined) {
      resolved.image_width = msg.image_width;
    }
    else {
      resolved.image_width = 0.0
    }

    if (msg.image_height !== undefined) {
      resolved.image_height = msg.image_height;
    }
    else {
      resolved.image_height = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: TopCamDataSrvRequest,
  Response: TopCamDataSrvResponse,
  md5sum() { return 'eed9195ebff712c1941314f098f70944'; },
  datatype() { return 'deepracer_simulation_environment/TopCamDataSrv'; }
};
