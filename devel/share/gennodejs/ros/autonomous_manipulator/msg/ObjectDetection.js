// Auto-generated. Do not edit!

// (in-package autonomous_manipulator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ObjectDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.color = null;
      this.u = null;
      this.v = null;
      this.area_px = null;
      this.position = null;
      this.ray_origin = null;
      this.ray_direction = null;
    }
    else {
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = '';
      }
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = 0.0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0.0;
      }
      if (initObj.hasOwnProperty('area_px')) {
        this.area_px = initObj.area_px
      }
      else {
        this.area_px = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('ray_origin')) {
        this.ray_origin = initObj.ray_origin
      }
      else {
        this.ray_origin = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('ray_direction')) {
        this.ray_direction = initObj.ray_direction
      }
      else {
        this.ray_direction = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectDetection
    // Serialize message field [color]
    bufferOffset = _serializer.string(obj.color, buffer, bufferOffset);
    // Serialize message field [u]
    bufferOffset = _serializer.float32(obj.u, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.float32(obj.v, buffer, bufferOffset);
    // Serialize message field [area_px]
    bufferOffset = _serializer.float32(obj.area_px, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [ray_origin]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.ray_origin, buffer, bufferOffset);
    // Serialize message field [ray_direction]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.ray_direction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectDetection
    let len;
    let data = new ObjectDetection(null);
    // Deserialize message field [color]
    data.color = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [u]
    data.u = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [area_px]
    data.area_px = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [ray_origin]
    data.ray_origin = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [ray_direction]
    data.ray_direction = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.color);
    return length + 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'autonomous_manipulator/ObjectDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb50bd15ebca29cc2431eb4b3677e8aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # One colour blob detected by the perception node.
    string color                       # orange | green | white | yellow
    float32 u                          # image coordinates of the (top-face) centroid [px]
    float32 v
    float32 area_px                    # blob area [px]
    geometry_msgs/Point position       # object position on the ground-plane assumption (plane z = plane_z of the array)
    geometry_msgs/Point ray_origin     # camera centre in the world frame
    geometry_msgs/Vector3 ray_direction  # unit ray through the centroid, world frame
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
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
    const resolved = new ObjectDetection(null);
    if (msg.color !== undefined) {
      resolved.color = msg.color;
    }
    else {
      resolved.color = ''
    }

    if (msg.u !== undefined) {
      resolved.u = msg.u;
    }
    else {
      resolved.u = 0.0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0.0
    }

    if (msg.area_px !== undefined) {
      resolved.area_px = msg.area_px;
    }
    else {
      resolved.area_px = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.ray_origin !== undefined) {
      resolved.ray_origin = geometry_msgs.msg.Point.Resolve(msg.ray_origin)
    }
    else {
      resolved.ray_origin = new geometry_msgs.msg.Point()
    }

    if (msg.ray_direction !== undefined) {
      resolved.ray_direction = geometry_msgs.msg.Vector3.Resolve(msg.ray_direction)
    }
    else {
      resolved.ray_direction = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = ObjectDetection;
