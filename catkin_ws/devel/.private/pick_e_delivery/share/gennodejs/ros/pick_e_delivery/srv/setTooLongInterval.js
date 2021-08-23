// Auto-generated. Do not edit!

// (in-package pick_e_delivery.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class setTooLongIntervalRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.period = null;
    }
    else {
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setTooLongIntervalRequest
    // Serialize message field [period]
    bufferOffset = _serializer.float32(obj.period, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setTooLongIntervalRequest
    let len;
    let data = new setTooLongIntervalRequest(null);
    // Deserialize message field [period]
    data.period = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pick_e_delivery/setTooLongIntervalRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bfcf350fbd9f1aad223e564a8c1f0dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 period
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setTooLongIntervalRequest(null);
    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = 0.0
    }

    return resolved;
    }
};

class setTooLongIntervalResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.period = null;
    }
    else {
      if (initObj.hasOwnProperty('period')) {
        this.period = initObj.period
      }
      else {
        this.period = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setTooLongIntervalResponse
    // Serialize message field [period]
    bufferOffset = _serializer.float32(obj.period, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setTooLongIntervalResponse
    let len;
    let data = new setTooLongIntervalResponse(null);
    // Deserialize message field [period]
    data.period = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'pick_e_delivery/setTooLongIntervalResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bfcf350fbd9f1aad223e564a8c1f0dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 period
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setTooLongIntervalResponse(null);
    if (msg.period !== undefined) {
      resolved.period = msg.period;
    }
    else {
      resolved.period = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: setTooLongIntervalRequest,
  Response: setTooLongIntervalResponse,
  md5sum() { return '32162b1483389f970edd3323d63a7978'; },
  datatype() { return 'pick_e_delivery/setTooLongInterval'; }
};
