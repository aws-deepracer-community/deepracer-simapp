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

let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GetVisualsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_names = null;
      this.visual_names = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetVisualsRequest
    // Serialize message field [link_names]
    bufferOffset = _arraySerializer.string(obj.link_names, buffer, bufferOffset, null);
    // Serialize message field [visual_names]
    bufferOffset = _arraySerializer.string(obj.visual_names, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetVisualsRequest
    let len;
    let data = new GetVisualsRequest(null);
    // Deserialize message field [link_names]
    data.link_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [visual_names]
    data.visual_names = _arrayDeserializer.string(buffer, bufferOffset, null)
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
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetVisualsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3c4dd005a7b7d78ec90c1c1b20dd7a43';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] link_names
    string[] visual_names
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetVisualsRequest(null);
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

    return resolved;
    }
};

class GetVisualsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_names = null;
      this.visual_names = null;
      this.ambients = null;
      this.diffuses = null;
      this.speculars = null;
      this.emissives = null;
      this.transparencies = null;
      this.visibles = null;
      this.geometry_types = null;
      this.mesh_geom_filenames = null;
      this.mesh_geom_scales = null;
      this.poses = null;
      this.success = null;
      this.status_message = null;
      this.status = null;
      this.messages = null;
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
      if (initObj.hasOwnProperty('transparencies')) {
        this.transparencies = initObj.transparencies
      }
      else {
        this.transparencies = [];
      }
      if (initObj.hasOwnProperty('visibles')) {
        this.visibles = initObj.visibles
      }
      else {
        this.visibles = [];
      }
      if (initObj.hasOwnProperty('geometry_types')) {
        this.geometry_types = initObj.geometry_types
      }
      else {
        this.geometry_types = [];
      }
      if (initObj.hasOwnProperty('mesh_geom_filenames')) {
        this.mesh_geom_filenames = initObj.mesh_geom_filenames
      }
      else {
        this.mesh_geom_filenames = [];
      }
      if (initObj.hasOwnProperty('mesh_geom_scales')) {
        this.mesh_geom_scales = initObj.mesh_geom_scales
      }
      else {
        this.mesh_geom_scales = [];
      }
      if (initObj.hasOwnProperty('poses')) {
        this.poses = initObj.poses
      }
      else {
        this.poses = [];
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
    // Serializes a message object of type GetVisualsResponse
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
    // Serialize message field [transparencies]
    bufferOffset = _arraySerializer.float64(obj.transparencies, buffer, bufferOffset, null);
    // Serialize message field [visibles]
    bufferOffset = _arraySerializer.int8(obj.visibles, buffer, bufferOffset, null);
    // Serialize message field [geometry_types]
    bufferOffset = _arraySerializer.uint16(obj.geometry_types, buffer, bufferOffset, null);
    // Serialize message field [mesh_geom_filenames]
    bufferOffset = _arraySerializer.string(obj.mesh_geom_filenames, buffer, bufferOffset, null);
    // Serialize message field [mesh_geom_scales]
    // Serialize the length for message field [mesh_geom_scales]
    bufferOffset = _serializer.uint32(obj.mesh_geom_scales.length, buffer, bufferOffset);
    obj.mesh_geom_scales.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [poses]
    // Serialize the length for message field [poses]
    bufferOffset = _serializer.uint32(obj.poses.length, buffer, bufferOffset);
    obj.poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
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
    //deserializes a message object of type GetVisualsResponse
    let len;
    let data = new GetVisualsResponse(null);
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
    // Deserialize message field [transparencies]
    data.transparencies = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [visibles]
    data.visibles = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [geometry_types]
    data.geometry_types = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [mesh_geom_filenames]
    data.mesh_geom_filenames = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [mesh_geom_scales]
    // Deserialize array length for message field [mesh_geom_scales]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.mesh_geom_scales = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.mesh_geom_scales[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [poses]
    // Deserialize array length for message field [poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
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
    length += 8 * object.transparencies.length;
    length += object.visibles.length;
    length += 2 * object.geometry_types.length;
    object.mesh_geom_filenames.forEach((val) => {
      length += 4 + val.length;
    });
    length += 24 * object.mesh_geom_scales.length;
    length += 56 * object.poses.length;
    length += object.status_message.length;
    length += object.status.length;
    object.messages.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 61;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetVisualsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7ea1d75463da94ce6f8ea172b14f758a';
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
    float64[] transparencies
    int8[] visibles
    uint16[] geometry_types
    string[] mesh_geom_filenames
    geometry_msgs/Vector3[] mesh_geom_scales
    geometry_msgs/Pose[] poses
    bool success
    string status_message
    int8[] status
    string[] messages
    
    ================================================================================
    MSG: std_msgs/ColorRGBA
    float32 r
    float32 g
    float32 b
    float32 a
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetVisualsResponse(null);
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

    if (msg.transparencies !== undefined) {
      resolved.transparencies = msg.transparencies;
    }
    else {
      resolved.transparencies = []
    }

    if (msg.visibles !== undefined) {
      resolved.visibles = msg.visibles;
    }
    else {
      resolved.visibles = []
    }

    if (msg.geometry_types !== undefined) {
      resolved.geometry_types = msg.geometry_types;
    }
    else {
      resolved.geometry_types = []
    }

    if (msg.mesh_geom_filenames !== undefined) {
      resolved.mesh_geom_filenames = msg.mesh_geom_filenames;
    }
    else {
      resolved.mesh_geom_filenames = []
    }

    if (msg.mesh_geom_scales !== undefined) {
      resolved.mesh_geom_scales = new Array(msg.mesh_geom_scales.length);
      for (let i = 0; i < resolved.mesh_geom_scales.length; ++i) {
        resolved.mesh_geom_scales[i] = geometry_msgs.msg.Vector3.Resolve(msg.mesh_geom_scales[i]);
      }
    }
    else {
      resolved.mesh_geom_scales = []
    }

    if (msg.poses !== undefined) {
      resolved.poses = new Array(msg.poses.length);
      for (let i = 0; i < resolved.poses.length; ++i) {
        resolved.poses[i] = geometry_msgs.msg.Pose.Resolve(msg.poses[i]);
      }
    }
    else {
      resolved.poses = []
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
  Request: GetVisualsRequest,
  Response: GetVisualsResponse,
  md5sum() { return 'aae1fc3904e0c0f096374889f5dbd21f'; },
  datatype() { return 'deepracer_msgs/GetVisuals'; }
};
