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

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class VideoMetricsSrvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VideoMetricsSrvRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VideoMetricsSrvRequest
    let len;
    let data = new VideoMetricsSrvRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation_environment/VideoMetricsSrvRequest';
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
    const resolved = new VideoMetricsSrvRequest(null);
    return resolved;
    }
};

class VideoMetricsSrvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lap_counter = null;
      this.completion_percentage = null;
      this.reset_counter = null;
      this.throttle = null;
      this.steering = null;
      this.best_lap_time = null;
      this.total_evaluation_time = null;
      this.done = null;
      this.x = null;
      this.y = null;
      this.object_locations = null;
    }
    else {
      if (initObj.hasOwnProperty('lap_counter')) {
        this.lap_counter = initObj.lap_counter
      }
      else {
        this.lap_counter = 0.0;
      }
      if (initObj.hasOwnProperty('completion_percentage')) {
        this.completion_percentage = initObj.completion_percentage
      }
      else {
        this.completion_percentage = 0.0;
      }
      if (initObj.hasOwnProperty('reset_counter')) {
        this.reset_counter = initObj.reset_counter
      }
      else {
        this.reset_counter = 0;
      }
      if (initObj.hasOwnProperty('throttle')) {
        this.throttle = initObj.throttle
      }
      else {
        this.throttle = 0.0;
      }
      if (initObj.hasOwnProperty('steering')) {
        this.steering = initObj.steering
      }
      else {
        this.steering = 0.0;
      }
      if (initObj.hasOwnProperty('best_lap_time')) {
        this.best_lap_time = initObj.best_lap_time
      }
      else {
        this.best_lap_time = 0.0;
      }
      if (initObj.hasOwnProperty('total_evaluation_time')) {
        this.total_evaluation_time = initObj.total_evaluation_time
      }
      else {
        this.total_evaluation_time = 0.0;
      }
      if (initObj.hasOwnProperty('done')) {
        this.done = initObj.done
      }
      else {
        this.done = false;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('object_locations')) {
        this.object_locations = initObj.object_locations
      }
      else {
        this.object_locations = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VideoMetricsSrvResponse
    // Serialize message field [lap_counter]
    bufferOffset = _serializer.float32(obj.lap_counter, buffer, bufferOffset);
    // Serialize message field [completion_percentage]
    bufferOffset = _serializer.float32(obj.completion_percentage, buffer, bufferOffset);
    // Serialize message field [reset_counter]
    bufferOffset = _serializer.int32(obj.reset_counter, buffer, bufferOffset);
    // Serialize message field [throttle]
    bufferOffset = _serializer.float32(obj.throttle, buffer, bufferOffset);
    // Serialize message field [steering]
    bufferOffset = _serializer.float32(obj.steering, buffer, bufferOffset);
    // Serialize message field [best_lap_time]
    bufferOffset = _serializer.float32(obj.best_lap_time, buffer, bufferOffset);
    // Serialize message field [total_evaluation_time]
    bufferOffset = _serializer.float32(obj.total_evaluation_time, buffer, bufferOffset);
    // Serialize message field [done]
    bufferOffset = _serializer.bool(obj.done, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [object_locations]
    // Serialize the length for message field [object_locations]
    bufferOffset = _serializer.uint32(obj.object_locations.length, buffer, bufferOffset);
    obj.object_locations.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point32.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VideoMetricsSrvResponse
    let len;
    let data = new VideoMetricsSrvResponse(null);
    // Deserialize message field [lap_counter]
    data.lap_counter = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [completion_percentage]
    data.completion_percentage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reset_counter]
    data.reset_counter = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [throttle]
    data.throttle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [steering]
    data.steering = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [best_lap_time]
    data.best_lap_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [total_evaluation_time]
    data.total_evaluation_time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [done]
    data.done = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [object_locations]
    // Deserialize array length for message field [object_locations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.object_locations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.object_locations[i] = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.object_locations.length;
    return length + 41;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_simulation_environment/VideoMetricsSrvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ccf1b22780e2ca6fb64b60963df11d26';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 lap_counter
    float32 completion_percentage
    int32 reset_counter
    float32 throttle
    float32 steering
    float32 best_lap_time
    float32 total_evaluation_time
    bool done
    float32 x
    float32 y
    geometry_msgs/Point32[] object_locations
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VideoMetricsSrvResponse(null);
    if (msg.lap_counter !== undefined) {
      resolved.lap_counter = msg.lap_counter;
    }
    else {
      resolved.lap_counter = 0.0
    }

    if (msg.completion_percentage !== undefined) {
      resolved.completion_percentage = msg.completion_percentage;
    }
    else {
      resolved.completion_percentage = 0.0
    }

    if (msg.reset_counter !== undefined) {
      resolved.reset_counter = msg.reset_counter;
    }
    else {
      resolved.reset_counter = 0
    }

    if (msg.throttle !== undefined) {
      resolved.throttle = msg.throttle;
    }
    else {
      resolved.throttle = 0.0
    }

    if (msg.steering !== undefined) {
      resolved.steering = msg.steering;
    }
    else {
      resolved.steering = 0.0
    }

    if (msg.best_lap_time !== undefined) {
      resolved.best_lap_time = msg.best_lap_time;
    }
    else {
      resolved.best_lap_time = 0.0
    }

    if (msg.total_evaluation_time !== undefined) {
      resolved.total_evaluation_time = msg.total_evaluation_time;
    }
    else {
      resolved.total_evaluation_time = 0.0
    }

    if (msg.done !== undefined) {
      resolved.done = msg.done;
    }
    else {
      resolved.done = false
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.object_locations !== undefined) {
      resolved.object_locations = new Array(msg.object_locations.length);
      for (let i = 0; i < resolved.object_locations.length; ++i) {
        resolved.object_locations[i] = geometry_msgs.msg.Point32.Resolve(msg.object_locations[i]);
      }
    }
    else {
      resolved.object_locations = []
    }

    return resolved;
    }
};

module.exports = {
  Request: VideoMetricsSrvRequest,
  Response: VideoMetricsSrvResponse,
  md5sum() { return 'ccf1b22780e2ca6fb64b60963df11d26'; },
  datatype() { return 'deepracer_simulation_environment/VideoMetricsSrv'; }
};
