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

let gazebo_msgs = _finder('gazebo_msgs');

//-----------------------------------------------------------

class GetModelStatesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model_names = null;
      this.relative_entity_names = null;
    }
    else {
      if (initObj.hasOwnProperty('model_names')) {
        this.model_names = initObj.model_names
      }
      else {
        this.model_names = [];
      }
      if (initObj.hasOwnProperty('relative_entity_names')) {
        this.relative_entity_names = initObj.relative_entity_names
      }
      else {
        this.relative_entity_names = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetModelStatesRequest
    // Serialize message field [model_names]
    bufferOffset = _arraySerializer.string(obj.model_names, buffer, bufferOffset, null);
    // Serialize message field [relative_entity_names]
    bufferOffset = _arraySerializer.string(obj.relative_entity_names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetModelStatesRequest
    let len;
    let data = new GetModelStatesRequest(null);
    // Deserialize message field [model_names]
    data.model_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [relative_entity_names]
    data.relative_entity_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.model_names.forEach((val) => {
      length += 4 + val.length;
    });
    object.relative_entity_names.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetModelStatesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbf135e797ae47a2c0be5146ab829cc2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] model_names
    string[] relative_entity_names
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetModelStatesRequest(null);
    if (msg.model_names !== undefined) {
      resolved.model_names = msg.model_names;
    }
    else {
      resolved.model_names = []
    }

    if (msg.relative_entity_names !== undefined) {
      resolved.relative_entity_names = msg.relative_entity_names;
    }
    else {
      resolved.relative_entity_names = []
    }

    return resolved;
    }
};

class GetModelStatesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model_states = null;
      this.success = null;
      this.status_message = null;
      this.status = null;
      this.messages = null;
    }
    else {
      if (initObj.hasOwnProperty('model_states')) {
        this.model_states = initObj.model_states
      }
      else {
        this.model_states = [];
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
    // Serializes a message object of type GetModelStatesResponse
    // Serialize message field [model_states]
    // Serialize the length for message field [model_states]
    bufferOffset = _serializer.uint32(obj.model_states.length, buffer, bufferOffset);
    obj.model_states.forEach((val) => {
      bufferOffset = gazebo_msgs.msg.ModelState.serialize(val, buffer, bufferOffset);
    });
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
    //deserializes a message object of type GetModelStatesResponse
    let len;
    let data = new GetModelStatesResponse(null);
    // Deserialize message field [model_states]
    // Deserialize array length for message field [model_states]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.model_states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.model_states[i] = gazebo_msgs.msg.ModelState.deserialize(buffer, bufferOffset)
    }
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
    object.model_states.forEach((val) => {
      length += gazebo_msgs.msg.ModelState.getMessageSize(val);
    });
    length += object.status_message.length;
    length += object.status.length;
    object.messages.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetModelStatesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d116e933124d536050bdc100783f312';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    gazebo_msgs/ModelState[] model_states
    bool success
    string status_message
    int8[] status
    string[] messages
    
    ================================================================================
    MSG: gazebo_msgs/ModelState
    # Set Gazebo Model pose and twist
    string model_name           # model to set state (pose and twist)
    geometry_msgs/Pose pose     # desired pose in reference frame
    geometry_msgs/Twist twist   # desired twist in reference frame
    string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
                                # leave empty or "world" or "map" defaults to world-frame
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetModelStatesResponse(null);
    if (msg.model_states !== undefined) {
      resolved.model_states = new Array(msg.model_states.length);
      for (let i = 0; i < resolved.model_states.length; ++i) {
        resolved.model_states[i] = gazebo_msgs.msg.ModelState.Resolve(msg.model_states[i]);
      }
    }
    else {
      resolved.model_states = []
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
  Request: GetModelStatesRequest,
  Response: GetModelStatesResponse,
  md5sum() { return '9f134a35b2aed42d29e57ebae260d3d6'; },
  datatype() { return 'deepracer_msgs/GetModelStates'; }
};
