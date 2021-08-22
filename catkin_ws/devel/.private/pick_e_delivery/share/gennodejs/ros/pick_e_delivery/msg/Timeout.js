// Auto-generated. Do not edit!

// (in-package pick_e_delivery.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Timeout {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.event = null;
    }
    else {
      if (initObj.hasOwnProperty('event')) {
        this.event = initObj.event
      }
      else {
        this.event = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Timeout
    // Serialize message field [event]
    bufferOffset = _serializer.string(obj.event, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Timeout
    let len;
    let data = new Timeout(null);
    // Deserialize message field [event]
    data.event = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.event.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pick_e_delivery/Timeout';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6aea470b1ed54075e83032ee4be16538';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string event
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Timeout(null);
    if (msg.event !== undefined) {
      resolved.event = msg.event;
    }
    else {
      resolved.event = ''
    }

    return resolved;
    }
};

module.exports = Timeout;
