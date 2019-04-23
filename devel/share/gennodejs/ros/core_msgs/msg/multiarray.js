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

class multiarray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.data = null;
      this.cols = null;
      this.rows = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
      if (initObj.hasOwnProperty('cols')) {
        this.cols = initObj.cols
      }
      else {
        this.cols = 0;
      }
      if (initObj.hasOwnProperty('rows')) {
        this.rows = initObj.rows
      }
      else {
        this.rows = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type multiarray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _arraySerializer.int8(obj.data, buffer, bufferOffset, null);
    // Serialize message field [cols]
    bufferOffset = _serializer.int32(obj.cols, buffer, bufferOffset);
    // Serialize message field [rows]
    bufferOffset = _serializer.int32(obj.rows, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type multiarray
    let len;
    let data = new multiarray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _arrayDeserializer.int8(buffer, bufferOffset, null)
    // Deserialize message field [cols]
    data.cols = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rows]
    data.rows = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.data.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'core_msgs/multiarray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f8105e16e9e9d9f45d4acb38630a37d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int8[] data
    int32 cols
    int32 rows
    
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
    const resolved = new multiarray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    if (msg.cols !== undefined) {
      resolved.cols = msg.cols;
    }
    else {
      resolved.cols = 0
    }

    if (msg.rows !== undefined) {
      resolved.rows = msg.rows;
    }
    else {
      resolved.rows = 0
    }

    return resolved;
    }
};

module.exports = multiarray;
