// Auto-generated. Do not edit!

// (in-package deepracer_simulation_environment.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class AgentRewardData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.agent_name = null;
      this.action = null;
      this.reward = null;
      this.action_space_len = null;
      this.speed_list = null;
      this.steering_angle_list = null;
      this.image = null;
    }
    else {
      if (initObj.hasOwnProperty('agent_name')) {
        this.agent_name = initObj.agent_name
      }
      else {
        this.agent_name = '';
      }
      if (initObj.hasOwnProperty('action')) {
        this.action = initObj.action
      }
      else {
        this.action = 0;
      }
      if (initObj.hasOwnProperty('reward')) {
        this.reward = initObj.reward
      }
      else {
        this.reward = 0.0;
      }
      if (initObj.hasOwnProperty('action_space_len')) {
        this.action_space_len = initObj.action_space_len
      }
      else {
        this.action_space_len = 0;
      }
      if (initObj.hasOwnProperty('speed_list')) {
        this.speed_list = initObj.speed_list
      }
      else {
        this.speed_list = [];
      }
      if (initObj.hasOwnProperty('steering_angle_list')) {
        this.steering_angle_list = initObj.steering_angle_list
      }
      else {
        this.steering_angle_list = [];
      }
      if (initObj.hasOwnProperty('image')) {
        this.image = initObj.image
      }
      else {
        this.image = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AgentRewardData
    // Serialize message field [agent_name]
    bufferOffset = _serializer.string(obj.agent_name, buffer, bufferOffset);
    // Serialize message field [action]
    bufferOffset = _serializer.int8(obj.action, buffer, bufferOffset);
    // Serialize message field [reward]
    bufferOffset = _serializer.float64(obj.reward, buffer, bufferOffset);
    // Serialize message field [action_space_len]
    bufferOffset = _serializer.int8(obj.action_space_len, buffer, bufferOffset);
    // Serialize message field [speed_list]
    bufferOffset = _arraySerializer.string(obj.speed_list, buffer, bufferOffset, null);
    // Serialize message field [steering_angle_list]
    bufferOffset = _arraySerializer.string(obj.steering_angle_list, buffer, bufferOffset, null);
    // Serialize message field [image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.image, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AgentRewardData
    let len;
    let data = new AgentRewardData(null);
    // Deserialize message field [agent_name]
    data.agent_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [action]
    data.action = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [reward]
    data.reward = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [action_space_len]
    data.action_space_len = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [speed_list]
    data.speed_list = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [steering_angle_list]
    data.steering_angle_list = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [image]
    data.image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.agent_name.length;
    object.speed_list.forEach((val) => {
      length += 4 + val.length;
    });
    object.steering_angle_list.forEach((val) => {
      length += 4 + val.length;
    });
    length += sensor_msgs.msg.Image.getMessageSize(object.image);
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'deepracer_simulation_environment/AgentRewardData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'af5e8658bf60dab3f19408bd4ad157dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string agent_name
    int8 action
    float64 reward
    int8 action_space_len
    string[] speed_list
    string[] steering_angle_list
    sensor_msgs/Image image
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AgentRewardData(null);
    if (msg.agent_name !== undefined) {
      resolved.agent_name = msg.agent_name;
    }
    else {
      resolved.agent_name = ''
    }

    if (msg.action !== undefined) {
      resolved.action = msg.action;
    }
    else {
      resolved.action = 0
    }

    if (msg.reward !== undefined) {
      resolved.reward = msg.reward;
    }
    else {
      resolved.reward = 0.0
    }

    if (msg.action_space_len !== undefined) {
      resolved.action_space_len = msg.action_space_len;
    }
    else {
      resolved.action_space_len = 0
    }

    if (msg.speed_list !== undefined) {
      resolved.speed_list = msg.speed_list;
    }
    else {
      resolved.speed_list = []
    }

    if (msg.steering_angle_list !== undefined) {
      resolved.steering_angle_list = msg.steering_angle_list;
    }
    else {
      resolved.steering_angle_list = []
    }

    if (msg.image !== undefined) {
      resolved.image = sensor_msgs.msg.Image.Resolve(msg.image)
    }
    else {
      resolved.image = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

module.exports = AgentRewardData;
