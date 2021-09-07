// Auto-generated. Do not edit!

// (in-package srrg2_core_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PlannerStatusMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.status = null;
      this.a_star_status = null;
      this.path_type = null;
      this.cost_to_global_goal = null;
      this.distance_to_global_goal = null;
      this.distance_to_local_goal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
      if (initObj.hasOwnProperty('a_star_status')) {
        this.a_star_status = initObj.a_star_status
      }
      else {
        this.a_star_status = 0;
      }
      if (initObj.hasOwnProperty('path_type')) {
        this.path_type = initObj.path_type
      }
      else {
        this.path_type = 0;
      }
      if (initObj.hasOwnProperty('cost_to_global_goal')) {
        this.cost_to_global_goal = initObj.cost_to_global_goal
      }
      else {
        this.cost_to_global_goal = 0.0;
      }
      if (initObj.hasOwnProperty('distance_to_global_goal')) {
        this.distance_to_global_goal = initObj.distance_to_global_goal
      }
      else {
        this.distance_to_global_goal = 0.0;
      }
      if (initObj.hasOwnProperty('distance_to_local_goal')) {
        this.distance_to_local_goal = initObj.distance_to_local_goal
      }
      else {
        this.distance_to_local_goal = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlannerStatusMessage
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    // Serialize message field [a_star_status]
    bufferOffset = _serializer.int32(obj.a_star_status, buffer, bufferOffset);
    // Serialize message field [path_type]
    bufferOffset = _serializer.int32(obj.path_type, buffer, bufferOffset);
    // Serialize message field [cost_to_global_goal]
    bufferOffset = _serializer.float32(obj.cost_to_global_goal, buffer, bufferOffset);
    // Serialize message field [distance_to_global_goal]
    bufferOffset = _serializer.float32(obj.distance_to_global_goal, buffer, bufferOffset);
    // Serialize message field [distance_to_local_goal]
    bufferOffset = _serializer.float32(obj.distance_to_local_goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlannerStatusMessage
    let len;
    let data = new PlannerStatusMessage(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [a_star_status]
    data.a_star_status = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [path_type]
    data.path_type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [cost_to_global_goal]
    data.cost_to_global_goal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance_to_global_goal]
    data.distance_to_global_goal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance_to_local_goal]
    data.distance_to_local_goal = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.status.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'srrg2_core_ros/PlannerStatusMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cde3a14282df6b919fe6c0fcba349400';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #gg mesage for the planner status, shot continuosly by the planner
    
    Header   header
    string   status           #status of the planner ("idle", "moving", "invalid_goal", "unreachable")
    int32    a_star_status    #result of the enum for the local path search
    int32    path_type        #type of path 0: gradient, 1: grid
    float32  cost_to_global_goal     #value of the cost function at the current location
    float32  distance_to_global_goal #distance to the current location [meters]
    float32  distance_to_local_goal #distance to the current location [meters]
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlannerStatusMessage(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    if (msg.a_star_status !== undefined) {
      resolved.a_star_status = msg.a_star_status;
    }
    else {
      resolved.a_star_status = 0
    }

    if (msg.path_type !== undefined) {
      resolved.path_type = msg.path_type;
    }
    else {
      resolved.path_type = 0
    }

    if (msg.cost_to_global_goal !== undefined) {
      resolved.cost_to_global_goal = msg.cost_to_global_goal;
    }
    else {
      resolved.cost_to_global_goal = 0.0
    }

    if (msg.distance_to_global_goal !== undefined) {
      resolved.distance_to_global_goal = msg.distance_to_global_goal;
    }
    else {
      resolved.distance_to_global_goal = 0.0
    }

    if (msg.distance_to_local_goal !== undefined) {
      resolved.distance_to_local_goal = msg.distance_to_local_goal;
    }
    else {
      resolved.distance_to_local_goal = 0.0
    }

    return resolved;
    }
};

module.exports = PlannerStatusMessage;
