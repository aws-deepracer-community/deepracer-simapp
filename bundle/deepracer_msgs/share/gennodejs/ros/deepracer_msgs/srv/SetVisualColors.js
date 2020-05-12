// Auto-generated. Do not edit!

// (in-package deepracer_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetVisualColorsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_names = null;
      this.visual_names = null;
      this.ambients = null;
      this.diffuses = null;
      this.speculars = null;
      this.emissives = null;
      this.block = null;
    }
    else {
      if (initObj.hasOwnProperty('link_names')) {
        this.link_names = initObj.link_names
      }
      else {
        this.link_names = [];
      }
      if (initObj.hasOwnProperty('visual_names')) {
        this.visual_names = initObj.visual_names
      }
      else {
        this.visual_names = [];
      }
      if (initObj.hasOwnProperty('ambients')) {
        this.ambients = initObj.ambients
      }
      else {
        this.ambients = [];
      }
      if (initObj.hasOwnProperty('diffuses')) {
        this.diffuses = initObj.diffuses
      }
      else {
        this.diffuses = [];
      }
      if (initObj.hasOwnProperty('speculars')) {
        this.speculars = initObj.speculars
      }
      else {
        this.speculars = [];
      }
      if (initObj.hasOwnProperty('emissives')) {
        this.emissives = initObj.emissives
      }
      else {
        this.emissives = [];
      }
      if (initObj.hasOwnProperty('block')) {
        this.block = initObj.block
      }
      else {
        this.block = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetVisualColorsRequest
    // Serialize message field [link_names]
    bufferOffset = _arraySerializer.string(obj.link_names, buffer, bufferOffset, null);
    // Serialize message field [visual_names]
    bufferOffset = _arraySerializer.string(obj.visual_names, buffer, bufferOffset, null);
    // Serialize message field [ambients]
    // Serialize the length for message field [ambients]
    bufferOffset = _serializer.uint32(obj.ambients.length, buffer, bufferOffset);
    obj.ambients.forEach((val) => {
      bufferOffset = std_msgs.msg.ColorRGBA.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [diffuses]
    // Serialize the length for message field [diffuses]
    bufferOffset = _serializer.uint32(obj.diffuses.length, buffer, bufferOffset);
    obj.diffuses.forEach((val) => {
      bufferOffset = std_msgs.msg.ColorRGBA.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [speculars]
    // Serialize the length for message field [speculars]
    bufferOffset = _serializer.uint32(obj.speculars.length, buffer, bufferOffset);
    obj.speculars.forEach((val) => {
      bufferOffset = std_msgs.msg.ColorRGBA.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [emissives]
    // Serialize the length for message field [emissives]
    bufferOffset = _serializer.uint32(obj.emissives.length, buffer, bufferOffset);
    obj.emissives.forEach((val) => {
      bufferOffset = std_msgs.msg.ColorRGBA.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [block]
    bufferOffset = _serializer.bool(obj.block, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetVisualColorsRequest
    let len;
    let data = new SetVisualColorsRequest(null);
    // Deserialize message field [link_names]
    data.link_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [visual_names]
    data.visual_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [ambients]
    // Deserialize array length for message field [ambients]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ambients = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ambients[i] = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [diffuses]
    // Deserialize array length for message field [diffuses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.diffuses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.diffuses[i] = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [speculars]
    // Deserialize array length for message field [speculars]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.speculars = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.speculars[i] = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [emissives]
    // Deserialize array length for message field [emissives]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.emissives = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.emissives[i] = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [block]
    data.block = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.link_names.forEach((val) => {
      length += 4 + val.length;
    });
    object.visual_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 16 * object.ambients.length;
    length += 16 * object.diffuses.length;
    length += 16 * object.speculars.length;
    length += 16 * object.emissives.length;
    return length + 25;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/SetVisualColorsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '52f74642913b71ec583802100623aef1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] link_names
    string[] visual_names
    std_msgs/ColorRGBA[] ambients
    std_msgs/ColorRGBA[] diffuses
    std_msgs/ColorRGBA[] speculars
    std_msgs/ColorRGBA[] emissives
    bool block
    
    ================================================================================
    MSG: std_msgs/ColorRGBA
    float32 r
    float32 g
    float32 b
    float32 a
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetVisualColorsRequest(null);
    if (msg.link_names !== undefined) {
      resolved.link_names = msg.link_names;
    }
    else {
      resolved.link_names = []
    }

    if (msg.visual_names !== undefined) {
      resolved.visual_names = msg.visual_names;
    }
    else {
      resolved.visual_names = []
    }

    if (msg.ambients !== undefined) {
      resolved.ambients = new Array(msg.ambients.length);
      for (let i = 0; i < resolved.ambients.length; ++i) {
        resolved.ambients[i] = std_msgs.msg.ColorRGBA.Resolve(msg.ambients[i]);
      }
    }
    else {
      resolved.ambients = []
    }

    if (msg.diffuses !== undefined) {
      resolved.diffuses = new Array(msg.diffuses.length);
      for (let i = 0; i < resolved.diffuses.length; ++i) {
        resolved.diffuses[i] = std_msgs.msg.ColorRGBA.Resolve(msg.diffuses[i]);
      }
    }
    else {
      resolved.diffuses = []
    }

    if (msg.speculars !== undefined) {
      resolved.speculars = new Array(msg.speculars.length);
      for (let i = 0; i < resolved.speculars.length; ++i) {
        resolved.speculars[i] = std_msgs.msg.ColorRGBA.Resolve(msg.speculars[i]);
      }
    }
    else {
      resolved.speculars = []
    }

    if (msg.emissives !== undefined) {
      resolved.emissives = new Array(msg.emissives.length);
      for (let i = 0; i < resolved.emissives.length; ++i) {
        resolved.emissives[i] = std_msgs.msg.ColorRGBA.Resolve(msg.emissives[i]);
      }
    }
    else {
      resolved.emissives = []
    }

    if (msg.block !== undefined) {
      resolved.block = msg.block;
    }
    else {
      resolved.block = false
    }

    return resolved;
    }
};

class SetVisualColorsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.status_message = null;
      this.status = null;
      this.messages = null;
    }
    else {
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
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = [];
      }
      if (initObj.hasOwnProperty('messages')) {
        this.messages = initObj.messages
      }
      else {
        this.messages = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetVisualColorsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [status_message]
    bufferOffset = _serializer.string(obj.status_message, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _arraySerializer.int8(obj.status, buffer, bufferOffset, null);
    // Serialize message field [messages]
    bufferOffset = _arraySerializer.string(obj.messages, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetVisualColorsResponse
    let len;
    let data = new SetVisualColorsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status_message]
    data.status_message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [messages]
    data.messages = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.status_message.length;
    length += object.status.length;
    object.messages.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/SetVisualColorsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a0af81bf1f7c2eacb2693173f999072a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string status_message
    int8[] status
    string[] messages
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetVisualColorsResponse(null);
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

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = []
    }

    if (msg.messages !== undefined) {
      resolved.messages = msg.messages;
    }
    else {
      resolved.messages = []
    }

    return resolved;
    }
};

module.exports = {
  Request: SetVisualColorsRequest,
  Response: SetVisualColorsResponse,
  md5sum() { return '2b593b9606746213e8e7b797a0ade086'; },
  datatype() { return 'deepracer_msgs/SetVisualColors'; }
};
