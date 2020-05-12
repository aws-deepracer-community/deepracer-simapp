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

class GetVisualRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.link_name = null;
      this.visual_name = null;
    }
    else {
      if (initObj.hasOwnProperty('link_name')) {
        this.link_name = initObj.link_name
      }
      else {
        this.link_name = '';
      }
      if (initObj.hasOwnProperty('visual_name')) {
        this.visual_name = initObj.visual_name
      }
      else {
        this.visual_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetVisualRequest
    // Serialize message field [link_name]
    bufferOffset = _serializer.string(obj.link_name, buffer, bufferOffset);
    // Serialize message field [visual_name]
    bufferOffset = _serializer.string(obj.visual_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetVisualRequest
    let len;
    let data = new GetVisualRequest(null);
    // Deserialize message field [link_name]
    data.link_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [visual_name]
    data.visual_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.link_name.length;
    length += object.visual_name.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetVisualRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce98c74bcee1499d7171492d4a03720d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string link_name
    string visual_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetVisualRequest(null);
    if (msg.link_name !== undefined) {
      resolved.link_name = msg.link_name;
    }
    else {
      resolved.link_name = ''
    }

    if (msg.visual_name !== undefined) {
      resolved.visual_name = msg.visual_name;
    }
    else {
      resolved.visual_name = ''
    }

    return resolved;
    }
};

class GetVisualResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ambient = null;
      this.diffuse = null;
      this.specular = null;
      this.emissive = null;
      this.transparency = null;
      this.visible = null;
      this.geometry_type = null;
      this.mesh_geom_filename = null;
      this.mesh_geom_scale = null;
      this.pose = null;
      this.success = null;
      this.status_message = null;
    }
    else {
      if (initObj.hasOwnProperty('ambient')) {
        this.ambient = initObj.ambient
      }
      else {
        this.ambient = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('diffuse')) {
        this.diffuse = initObj.diffuse
      }
      else {
        this.diffuse = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('specular')) {
        this.specular = initObj.specular
      }
      else {
        this.specular = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('emissive')) {
        this.emissive = initObj.emissive
      }
      else {
        this.emissive = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('transparency')) {
        this.transparency = initObj.transparency
      }
      else {
        this.transparency = 0.0;
      }
      if (initObj.hasOwnProperty('visible')) {
        this.visible = initObj.visible
      }
      else {
        this.visible = false;
      }
      if (initObj.hasOwnProperty('geometry_type')) {
        this.geometry_type = initObj.geometry_type
      }
      else {
        this.geometry_type = 0;
      }
      if (initObj.hasOwnProperty('mesh_geom_filename')) {
        this.mesh_geom_filename = initObj.mesh_geom_filename
      }
      else {
        this.mesh_geom_filename = '';
      }
      if (initObj.hasOwnProperty('mesh_geom_scale')) {
        this.mesh_geom_scale = initObj.mesh_geom_scale
      }
      else {
        this.mesh_geom_scale = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
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
    // Serializes a message object of type GetVisualResponse
    // Serialize message field [ambient]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.ambient, buffer, bufferOffset);
    // Serialize message field [diffuse]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.diffuse, buffer, bufferOffset);
    // Serialize message field [specular]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.specular, buffer, bufferOffset);
    // Serialize message field [emissive]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.emissive, buffer, bufferOffset);
    // Serialize message field [transparency]
    bufferOffset = _serializer.float64(obj.transparency, buffer, bufferOffset);
    // Serialize message field [visible]
    bufferOffset = _serializer.bool(obj.visible, buffer, bufferOffset);
    // Serialize message field [geometry_type]
    bufferOffset = _serializer.uint16(obj.geometry_type, buffer, bufferOffset);
    // Serialize message field [mesh_geom_filename]
    bufferOffset = _serializer.string(obj.mesh_geom_filename, buffer, bufferOffset);
    // Serialize message field [mesh_geom_scale]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.mesh_geom_scale, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [status_message]
    bufferOffset = _serializer.string(obj.status_message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetVisualResponse
    let len;
    let data = new GetVisualResponse(null);
    // Deserialize message field [ambient]
    data.ambient = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [diffuse]
    data.diffuse = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [specular]
    data.specular = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [emissive]
    data.emissive = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [transparency]
    data.transparency = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [visible]
    data.visible = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [geometry_type]
    data.geometry_type = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [mesh_geom_filename]
    data.mesh_geom_filename = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mesh_geom_scale]
    data.mesh_geom_scale = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status_message]
    data.status_message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.mesh_geom_filename.length;
    length += object.status_message.length;
    return length + 164;
  }

  static datatype() {
    // Returns string type for a service object
    return 'deepracer_msgs/GetVisualResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc31fa8acbe9027829e3607a7f9e7454';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/ColorRGBA ambient
    std_msgs/ColorRGBA diffuse
    std_msgs/ColorRGBA specular
    std_msgs/ColorRGBA emissive
    float64 transparency
    bool visible
    uint16 geometry_type
    string mesh_geom_filename
    geometry_msgs/Vector3 mesh_geom_scale
    geometry_msgs/Pose pose
    bool success
    string status_message
    
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
    const resolved = new GetVisualResponse(null);
    if (msg.ambient !== undefined) {
      resolved.ambient = std_msgs.msg.ColorRGBA.Resolve(msg.ambient)
    }
    else {
      resolved.ambient = new std_msgs.msg.ColorRGBA()
    }

    if (msg.diffuse !== undefined) {
      resolved.diffuse = std_msgs.msg.ColorRGBA.Resolve(msg.diffuse)
    }
    else {
      resolved.diffuse = new std_msgs.msg.ColorRGBA()
    }

    if (msg.specular !== undefined) {
      resolved.specular = std_msgs.msg.ColorRGBA.Resolve(msg.specular)
    }
    else {
      resolved.specular = new std_msgs.msg.ColorRGBA()
    }

    if (msg.emissive !== undefined) {
      resolved.emissive = std_msgs.msg.ColorRGBA.Resolve(msg.emissive)
    }
    else {
      resolved.emissive = new std_msgs.msg.ColorRGBA()
    }

    if (msg.transparency !== undefined) {
      resolved.transparency = msg.transparency;
    }
    else {
      resolved.transparency = 0.0
    }

    if (msg.visible !== undefined) {
      resolved.visible = msg.visible;
    }
    else {
      resolved.visible = false
    }

    if (msg.geometry_type !== undefined) {
      resolved.geometry_type = msg.geometry_type;
    }
    else {
      resolved.geometry_type = 0
    }

    if (msg.mesh_geom_filename !== undefined) {
      resolved.mesh_geom_filename = msg.mesh_geom_filename;
    }
    else {
      resolved.mesh_geom_filename = ''
    }

    if (msg.mesh_geom_scale !== undefined) {
      resolved.mesh_geom_scale = geometry_msgs.msg.Vector3.Resolve(msg.mesh_geom_scale)
    }
    else {
      resolved.mesh_geom_scale = new geometry_msgs.msg.Vector3()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
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
  Request: GetVisualRequest,
  Response: GetVisualResponse,
  md5sum() { return 'addaab363bcf820667e503bbd31b4f3d'; },
  datatype() { return 'deepracer_msgs/GetVisual'; }
};
