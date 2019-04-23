// Auto-generated. Do not edit!

// (in-package core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ball_position_r {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.size_r = null;
      this.img_x_r = null;
      this.img_y_r = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('size_r')) {
        this.size_r = initObj.size_r
      }
      else {
        this.size_r = 0;
      }
      if (initObj.hasOwnProperty('img_x_r')) {
        this.img_x_r = initObj.img_x_r
      }
      else {
        this.img_x_r = [];
      }
      if (initObj.hasOwnProperty('img_y_r')) {
        this.img_y_r = initObj.img_y_r
      }
      else {
        this.img_y_r = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ball_position_r
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [size_r]
    bufferOffset = _serializer.int32(obj.size_r, buffer, bufferOffset);
    // Serialize message field [img_x_r]
    bufferOffset = _arraySerializer.float32(obj.img_x_r, buffer, bufferOffset, null);
    // Serialize message field [img_y_r]
    bufferOffset = _arraySerializer.float32(obj.img_y_r, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ball_position_r
    let len;
    let data = new ball_position_r(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [size_r]
    data.size_r = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [img_x_r]
    data.img_x_r = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [img_y_r]
    data.img_y_r = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.img_x_r.length;
    length += 4 * object.img_y_r.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'core_msgs/ball_position_r';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '726954b4f6c158c4b6d79aac965dd64e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 size_r
    float32[] img_x_r
    float32[] img_y_r
    
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
    const resolved = new ball_position_r(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.size_r !== undefined) {
      resolved.size_r = msg.size_r;
    }
    else {
      resolved.size_r = 0
    }

    if (msg.img_x_r !== undefined) {
      resolved.img_x_r = msg.img_x_r;
    }
    else {
      resolved.img_x_r = []
    }

    if (msg.img_y_r !== undefined) {
      resolved.img_y_r = msg.img_y_r;
    }
    else {
      resolved.img_y_r = []
    }

    return resolved;
    }
};

module.exports = ball_position_r;
