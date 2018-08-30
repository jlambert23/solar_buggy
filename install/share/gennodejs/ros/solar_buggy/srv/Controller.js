// Auto-generated. Do not edit!

// (in-package solar_buggy.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ControllerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerRequest
    let len;
    let data = new ControllerRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'solar_buggy/ControllerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerRequest(null);
    return resolved;
    }
};

class ControllerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.return_value = null;
    }
    else {
      if (initObj.hasOwnProperty('return_value')) {
        this.return_value = initObj.return_value
      }
      else {
        this.return_value = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerResponse
    // Serialize message field [return_value]
    bufferOffset = _serializer.string(obj.return_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerResponse
    let len;
    let data = new ControllerResponse(null);
    // Deserialize message field [return_value]
    data.return_value = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.return_value.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'solar_buggy/ControllerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b68c06a61da57812b519df2ba93f87ff';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string return_value
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerResponse(null);
    if (msg.return_value !== undefined) {
      resolved.return_value = msg.return_value;
    }
    else {
      resolved.return_value = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ControllerRequest,
  Response: ControllerResponse,
  md5sum() { return 'b68c06a61da57812b519df2ba93f87ff'; },
  datatype() { return 'solar_buggy/Controller'; }
};
