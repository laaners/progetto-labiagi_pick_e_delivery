// Auto-generated. Do not edit!

// (in-package srrg2_navigation_2d_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CollisionAvoiderStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cmd_vel_input = null;
      this.cmd_vel_output = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cmd_vel_input')) {
        this.cmd_vel_input = initObj.cmd_vel_input
      }
      else {
        this.cmd_vel_input = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('cmd_vel_output')) {
        this.cmd_vel_output = initObj.cmd_vel_output
      }
      else {
        this.cmd_vel_output = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CollisionAvoiderStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cmd_vel_input]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.cmd_vel_input, buffer, bufferOffset);
    // Serialize message field [cmd_vel_output]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.cmd_vel_output, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CollisionAvoiderStatus
    let len;
    let data = new CollisionAvoiderStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cmd_vel_input]
    data.cmd_vel_input = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [cmd_vel_output]
    data.cmd_vel_output = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.status.length;
    return length + 100;
  }

  static datatype() {
    // Returns string type for a message object
    return 'srrg2_navigation_2d_msgs/CollisionAvoiderStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '413ea057b6949dd5b85094f0459afba5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    #command input to the collision avoider
    geometry_msgs/Twist cmd_vel_input
    
    #command output by the collision avoider
    geometry_msgs/Twist cmd_vel_output
    
    
    # status can be either
    # "clear":    when the obstacles are fare enough and the avoider does not kick in
    # "adjusting": when the obstacles affect the control, but don't block the robot
    # "blocked":   when the robot stopped due to a dynamic obstacle
    
    string status
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
    string frame_id
    
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
    const resolved = new CollisionAvoiderStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cmd_vel_input !== undefined) {
      resolved.cmd_vel_input = geometry_msgs.msg.Twist.Resolve(msg.cmd_vel_input)
    }
    else {
      resolved.cmd_vel_input = new geometry_msgs.msg.Twist()
    }

    if (msg.cmd_vel_output !== undefined) {
      resolved.cmd_vel_output = geometry_msgs.msg.Twist.Resolve(msg.cmd_vel_output)
    }
    else {
      resolved.cmd_vel_output = new geometry_msgs.msg.Twist()
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    return resolved;
    }
};

module.exports = CollisionAvoiderStatus;
