// Auto-generated. Do not edit!

// (in-package robomaker_simulation_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Tag = require('../msg/Tag.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class AddTagsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tags = null;
    }
    else {
      if (initObj.hasOwnProperty('tags')) {
        this.tags = initObj.tags
      }
      else {
        this.tags = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddTagsRequest
    // Serialize message field [tags]
    // Serialize the length for message field [tags]
    bufferOffset = _serializer.uint32(obj.tags.length, buffer, bufferOffset);
    obj.tags.forEach((val) => {
      bufferOffset = Tag.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddTagsRequest
    let len;
    let data = new AddTagsRequest(null);
    // Deserialize message field [tags]
    // Deserialize array length for message field [tags]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tags = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tags[i] = Tag.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.tags.forEach((val) => {
      length += Tag.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robomaker_simulation_msgs/AddTagsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33d0d627ff06b11fa5f66485c6bef84d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    Tag[] tags
    
    ================================================================================
    MSG: robomaker_simulation_msgs/Tag
    # Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
    # SPDX-License-Identifier: Apache-2.0
    string key
    string value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddTagsRequest(null);
    if (msg.tags !== undefined) {
      resolved.tags = new Array(msg.tags.length);
      for (let i = 0; i < resolved.tags.length; ++i) {
        resolved.tags[i] = Tag.Resolve(msg.tags[i]);
      }
    }
    else {
      resolved.tags = []
    }

    return resolved;
    }
};

class AddTagsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AddTagsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AddTagsResponse
    let len;
    let data = new AddTagsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.message.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'robomaker_simulation_msgs/AddTagsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    bool success
    
    
    
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AddTagsResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: AddTagsRequest,
  Response: AddTagsResponse,
  md5sum() { return '637781bbb7edf1a52e9fe238f1ba7d6b'; },
  datatype() { return 'robomaker_simulation_msgs/AddTags'; }
};
