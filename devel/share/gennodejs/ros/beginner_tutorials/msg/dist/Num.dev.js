// Auto-generated. Do not edit!
// (in-package beginner_tutorials.msg)
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

var Num =
/*#__PURE__*/
function () {
  function Num() {
    var initObj = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : {};

    _classCallCheck(this, Num);

    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num = null;
    } else {
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num;
      } else {
        this.num = 0;
      }
    }
  }

  _createClass(Num, null, [{
    key: "serialize",
    value: function serialize(obj, buffer, bufferOffset) {
      // Serializes a message object of type Num
      // Serialize message field [num]
      bufferOffset = _serializer.int64(obj.num, buffer, bufferOffset);
      return bufferOffset;
    }
  }, {
    key: "deserialize",
    value: function deserialize(buffer) {
      var bufferOffset = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : [0];
      //deserializes a message object of type Num
      var len;
      var data = new Num(null); // Deserialize message field [num]

      data.num = _deserializer.int64(buffer, bufferOffset);
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
      // Returns string type for a message object
      return 'beginner_tutorials/Num';
    }
  }, {
    key: "md5sum",
    value: function md5sum() {
      //Returns md5sum for a message object
      return '57d3c40ec3ac3754af76a83e6e73127a';
    }
  }, {
    key: "messageDefinition",
    value: function messageDefinition() {
      // Returns full string definition for message
      return "\n    int64 num\n    \n    ";
    }
  }, {
    key: "Resolve",
    value: function Resolve(msg) {
      // deep-construct a valid message object instance of whatever was passed in
      if (_typeof(msg) !== 'object' || msg === null) {
        msg = {};
      }

      var resolved = new Num(null);

      if (msg.num !== undefined) {
        resolved.num = msg.num;
      } else {
        resolved.num = 0;
      }

      return resolved;
    }
  }]);

  return Num;
}();

;
module.exports = Num;