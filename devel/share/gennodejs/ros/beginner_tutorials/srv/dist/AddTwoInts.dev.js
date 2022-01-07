// Auto-generated. Do not edit!
// (in-package beginner_tutorials.srv)
"use strict";

function _typeof(obj) { if (typeof Symbol === "function" && typeof Symbol.iterator === "symbol") { _typeof = function _typeof(obj) { return typeof obj; }; } else { _typeof = function _typeof(obj) { return obj && typeof Symbol === "function" && obj.constructor === Symbol && obj !== Symbol.prototype ? "symbol" : typeof obj; }; } return _typeof(obj); }

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } }

function _createClass(Constructor, protoProps, staticProps) { if (protoProps) _defineProperties(Constructor.prototype, protoProps); if (staticProps) _defineProperties(Constructor, staticProps); return Constructor; }

var _serializer = _ros_msg_utils.Serialize;
var _arraySerializer = _serializer.Array;
var _deserializer = _ros_msg_utils.Deserialize;
var _arrayDeserializer = _deserializer.Array;
var _finder = _ros_msg_utils.Find;
var _getByteLength = _ros_msg_utils.getByteLength; //-----------------------------------------------------------
//-----------------------------------------------------------

var AddTwoIntsRequest =
/*#__PURE__*/
function () {
  function AddTwoIntsRequest() {
    var initObj = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : {};

    _classCallCheck(this, AddTwoIntsRequest);

    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.a = null;
      this.b = null;
    } else {
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a;
      } else {
        this.a = 0;
      }

      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b;
      } else {
        this.b = 0;
      }
    }
  }

  _createClass(AddTwoIntsRequest, null, [{
    key: "serialize",
    value: function serialize(obj, buffer, bufferOffset) {
      // Serializes a message object of type AddTwoIntsRequest
      // Serialize message field [a]
      bufferOffset = _serializer.int64(obj.a, buffer, bufferOffset); // Serialize message field [b]

      bufferOffset = _serializer.int64(obj.b, buffer, bufferOffset);
      return bufferOffset;
    }
  }, {
    key: "deserialize",
    value: function deserialize(buffer) {
      var bufferOffset = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : [0];
      //deserializes a message object of type AddTwoIntsRequest
      var len;
      var data = new AddTwoIntsRequest(null); // Deserialize message field [a]

      data.a = _deserializer.int64(buffer, bufferOffset); // Deserialize message field [b]

      data.b = _deserializer.int64(buffer, bufferOffset);
      return data;
    }
  }, {
    key: "getMessageSize",
    value: function getMessageSize(object) {
      return 16;
    }
  }, {
    key: "datatype",
    value: function datatype() {
      // Returns string type for a service object
      return 'beginner_tutorials/AddTwoIntsRequest';
    }
  }, {
    key: "md5sum",
    value: function md5sum() {
      //Returns md5sum for a message object
      return '36d09b846be0b371c5f190354dd3153e';
    }
  }, {
    key: "messageDefinition",
    value: function messageDefinition() {
      // Returns full string definition for message
      return "\n    int64 a\n    int64 b\n    \n    ";
    }
  }, {
    key: "Resolve",
    value: function Resolve(msg) {
      // deep-construct a valid message object instance of whatever was passed in
      if (_typeof(msg) !== 'object' || msg === null) {
        msg = {};
      }

      var resolved = new AddTwoIntsRequest(null);

      if (msg.a !== undefined) {
        resolved.a = msg.a;
      } else {
        resolved.a = 0;
      }

      if (msg.b !== undefined) {
        resolved.b = msg.b;
      } else {
        resolved.b = 0;
      }

      return resolved;
    }
  }]);

  return AddTwoIntsRequest;
}();

;

var AddTwoIntsResponse =
/*#__PURE__*/
function () {
  function AddTwoIntsResponse() {
    var initObj = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : {};

    _classCallCheck(this, AddTwoIntsResponse);

    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sum = null;
    } else {
      if (initObj.hasOwnProperty('sum')) {
        this.sum = initObj.sum;
      } else {
        this.sum = 0;
      }
    }
  }

  _createClass(AddTwoIntsResponse, null, [{
    key: "serialize",
    value: function serialize(obj, buffer, bufferOffset) {
      // Serializes a message object of type AddTwoIntsResponse
      // Serialize message field [sum]
      bufferOffset = _serializer.int64(obj.sum, buffer, bufferOffset);
      return bufferOffset;
    }
  }, {
    key: "deserialize",
    value: function deserialize(buffer) {
      var bufferOffset = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : [0];
      //deserializes a message object of type AddTwoIntsResponse
      var len;
      var data = new AddTwoIntsResponse(null); // Deserialize message field [sum]

      data.sum = _deserializer.int64(buffer, bufferOffset);
      return data;
    }
  }, {
    key: "getMessageSize",
    value: function getMessageSize(object) {
      return 8;
    }
  }, {
    key: "datatype",
    value: function datatype() {
      // Returns string type for a service object
      return 'beginner_tutorials/AddTwoIntsResponse';
    }
  }, {
    key: "md5sum",
    value: function md5sum() {
      //Returns md5sum for a message object
      return 'b88405221c77b1878a3cbbfff53428d7';
    }
  }, {
    key: "messageDefinition",
    value: function messageDefinition() {
      // Returns full string definition for message
      return "\n    int64 sum\n    \n    \n    ";
    }
  }, {
    key: "Resolve",
    value: function Resolve(msg) {
      // deep-construct a valid message object instance of whatever was passed in
      if (_typeof(msg) !== 'object' || msg === null) {
        msg = {};
      }

      var resolved = new AddTwoIntsResponse(null);

      if (msg.sum !== undefined) {
        resolved.sum = msg.sum;
      } else {
        resolved.sum = 0;
      }

      return resolved;
    }
  }]);

  return AddTwoIntsResponse;
}();

;
module.exports = {
  Request: AddTwoIntsRequest,
  Response: AddTwoIntsResponse,
  md5sum: function md5sum() {
    return '6a2e34150c00229791cc89ff309fff21';
  },
  datatype: function datatype() {
    return 'beginner_tutorials/AddTwoInts';
  }
};