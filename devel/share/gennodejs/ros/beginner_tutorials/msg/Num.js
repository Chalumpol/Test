// Auto-generated. Do not edit!

// (in-package beginner_tutorials.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class Num {
  constructor() {
    this.num = 0;
    this.first_name = '';
    this.last_name = '';
    this.age = 0;
    this.score = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type Num
    // Serialize message field [num]
    bufferInfo = _serializer.int64(obj.num, bufferInfo);
    // Serialize message field [first_name]
    bufferInfo = _serializer.string(obj.first_name, bufferInfo);
    // Serialize message field [last_name]
    bufferInfo = _serializer.string(obj.last_name, bufferInfo);
    // Serialize message field [age]
    bufferInfo = _serializer.uint8(obj.age, bufferInfo);
    // Serialize message field [score]
    bufferInfo = _serializer.uint32(obj.score, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type Num
    let tmp;
    let len;
    let data = new Num();
    // Deserialize message field [num]
    tmp = _deserializer.int64(buffer);
    data.num = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [first_name]
    tmp = _deserializer.string(buffer);
    data.first_name = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [last_name]
    tmp = _deserializer.string(buffer);
    data.last_name = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [age]
    tmp = _deserializer.uint8(buffer);
    data.age = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [score]
    tmp = _deserializer.uint32(buffer);
    data.score = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'beginner_tutorials/Num';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '379cbe21d44c48c3d9a5a30aa0ddaafb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 num
    string first_name
    string last_name
    uint8 age
    uint32 score
    `;
  }

};

module.exports = Num;
