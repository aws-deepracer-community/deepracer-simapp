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

class GetLinkStatesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_names = null;
      this.reference_frames = null;
    }
    else {
      if (initObj.hasOwnProperty('link_names')) {
        this.link_names = initObj.link_names
      }
      else {
        this.link_names = [];
      }
      if (initObj.hasOwnProperty('reference_frames')) {
        this.reference_frames = initObj.reference_frames
      }
      else {
        this.reference_frames = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetLinkStatesRequest
    // Serialize message field [link_names]
    bufferOffset = _arraySerializer.string(obj.link_names, buffer, bufferOffset, null);
    // Serialize message field [reference_frames]
    bufferOffset = _arraySerializer.string(obj.reference_frames, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetLinkStatesRequest
    let len;
    let data = new GetLinkStatesRequest(null);
    // Deserialize message field [link_names]
    data.link_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [reference_frames]
    data.reference_frames = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.link_names.forEach((val) => {
      length += 4 + val.length;
    });
    object.reference_frames.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetLinkStatesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '284df52820947bf6fdad4d9fe2eb3466';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] link_names
    
    string[] reference_frames
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetLinkStatesRequest(null);
    if (msg.link_names !== undefined) {
      resolved.link_names = msg.link_names;
    }
    else {
      resolved.link_names = []
    }

    if (msg.reference_frames !== undefined) {
      resolved.reference_frames = msg.reference_frames;
    }
    else {
      resolved.reference_frames = []
    }

    return resolved;
    }
};

class GetLinkStatesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_states = null;
      this.success = null;
      this.status_message = null;
      this.status = null;
      this.messages = null;
    }
    else {
      if (initObj.hasOwnProperty('link_states')) {
        this.link_states = initObj.link_states
      }
      else {
        this.link_states = [];
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
    // Serializes a message object of type GetLinkStatesResponse
    // Serialize message field [link_states]
    // Serialize the length for message field [link_states]
    bufferOffset = _serializer.uint32(obj.link_states.length, buffer, bufferOffset);
    obj.link_states.forEach((val) => {
      bufferOffset = gazebo_msgs.msg.LinkState.serialize(val, buffer, bufferOffset);
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
    //deserializes a message object of type GetLinkStatesResponse
    let len;
    let data = new GetLinkStatesResponse(null);
    // Deserialize message field [link_states]
    // Deserialize array length for message field [link_states]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.link_states = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.link_states[i] = gazebo_msgs.msg.LinkState.deserialize(buffer, bufferOffset)
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
    object.link_states.forEach((val) => {
      length += gazebo_msgs.msg.LinkState.getMessageSize(val);
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
    return 'deepracer_msgs/GetLinkStatesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3016f6bbe8575753300c5adc29c01810';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    gazebo_msgs/LinkState[] link_states
    bool success
    string status_message
    int8[] status
    string[] messages
    
    ================================================================================
    MSG: gazebo_msgs/LinkState
    # @todo: FIXME: sets pose and twist of a link.  All children link poses/twists of the URDF tree are not updated accordingly, but should be.
    string link_name            # link name, link_names are in gazebo scoped name notation, [model_name::body_name]
    geometry_msgs/Pose pose     # desired pose in reference frame
    geometry_msgs/Twist twist   # desired twist in reference frame
    string reference_frame      # set pose/twist relative to the frame of this link/body
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
    const resolved = new GetLinkStatesResponse(null);
    if (msg.link_states !== undefined) {
      resolved.link_states = new Array(msg.link_states.length);
      for (let i = 0; i < resolved.link_states.length; ++i) {
        resolved.link_states[i] = gazebo_msgs.msg.LinkState.Resolve(msg.link_states[i]);
      }
    }
    else {
      resolved.link_states = []
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
  Request: GetLinkStatesRequest,
  Response: GetLinkStatesResponse,
  md5sum() { return '1bc3add906c1ea529af2b9b4ae208741'; },
  datatype() { return 'deepracer_msgs/GetLinkStates'; }
};
