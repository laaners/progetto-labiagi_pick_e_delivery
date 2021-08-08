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

class PathFollowerStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_steps_to_goal = null;
      this.robot_pose_2d = null;
      this.local_target_2d = null;
      this.cmd_vel = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_steps_to_goal')) {
        this.num_steps_to_goal = initObj.num_steps_to_goal
      }
      else {
        this.num_steps_to_goal = 0;
      }
      if (initObj.hasOwnProperty('robot_pose_2d')) {
        this.robot_pose_2d = initObj.robot_pose_2d
      }
      else {
        this.robot_pose_2d = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('local_target_2d')) {
        this.local_target_2d = initObj.local_target_2d
      }
      else {
        this.local_target_2d = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('cmd_vel')) {
        this.cmd_vel = initObj.cmd_vel
      }
      else {
        this.cmd_vel = new geometry_msgs.msg.Twist();
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
    // Serializes a message object of type PathFollowerStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_steps_to_goal]
    bufferOffset = _serializer.int64(obj.num_steps_to_goal, buffer, bufferOffset);
    // Check that the constant length array field [robot_pose_2d] has the right length
    if (obj.robot_pose_2d.length !== 3) {
      throw new Error('Unable to serialize array field robot_pose_2d - length must be 3')
    }
    // Serialize message field [robot_pose_2d]
    bufferOffset = _arraySerializer.float32(obj.robot_pose_2d, buffer, bufferOffset, 3);
    // Check that the constant length array field [local_target_2d] has the right length
    if (obj.local_target_2d.length !== 3) {
      throw new Error('Unable to serialize array field local_target_2d - length must be 3')
    }
    // Serialize message field [local_target_2d]
    bufferOffset = _arraySerializer.float32(obj.local_target_2d, buffer, bufferOffset, 3);
    // Serialize message field [cmd_vel]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.cmd_vel, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathFollowerStatus
    let len;
    let data = new PathFollowerStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_steps_to_goal]
    data.num_steps_to_goal = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [robot_pose_2d]
    data.robot_pose_2d = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [local_target_2d]
    data.local_target_2d = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [cmd_vel]
    data.cmd_vel = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.status.length;
    return length + 84;
  }

  static datatype() {
    // Returns string type for a message object
    return 'srrg2_navigation_2d_msgs/PathFollowerStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ea28f7c3c138ef8732745d1ce16b8aaf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    #how many steps we need to reach the goal
    int64 num_steps_to_goal 
    
    #2D robot pose (x,y, theta)
    float32[3] robot_pose_2d
    
    #2D target of the robot in the local frame
    float32[3] local_target_2d
    
    #command output by the follower
    geometry_msgs/Twist cmd_vel
    
    # status can be either
    # "goal_reached":    when the path is empty
    # "initial_turning": when the robot initially rotates to align with the path
    # "cruising":        when the robot is tracking the path with the regular controller
    # "finalizing":      when the robot reached the final position, and adjusts the orientation
    # "error":           when the robot cannot reach the goal because no local target could be computed 
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
    const resolved = new PathFollowerStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_steps_to_goal !== undefined) {
      resolved.num_steps_to_goal = msg.num_steps_to_goal;
    }
    else {
      resolved.num_steps_to_goal = 0
    }

    if (msg.robot_pose_2d !== undefined) {
      resolved.robot_pose_2d = msg.robot_pose_2d;
    }
    else {
      resolved.robot_pose_2d = new Array(3).fill(0)
    }

    if (msg.local_target_2d !== undefined) {
      resolved.local_target_2d = msg.local_target_2d;
    }
    else {
      resolved.local_target_2d = new Array(3).fill(0)
    }

    if (msg.cmd_vel !== undefined) {
      resolved.cmd_vel = geometry_msgs.msg.Twist.Resolve(msg.cmd_vel)
    }
    else {
      resolved.cmd_vel = new geometry_msgs.msg.Twist()
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

module.exports = PathFollowerStatus;
