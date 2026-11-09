; Auto-generated. Do not edit!


(cl:in-package autonomous_manipulator-msg)


;//! \htmlinclude ObjectDetection.msg.html

(cl:defclass <ObjectDetection> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type cl:string
    :initform "")
   (u
    :reader u
    :initarg :u
    :type cl:float
    :initform 0.0)
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (area_px
    :reader area_px
    :initarg :area_px
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (ray_origin
    :reader ray_origin
    :initarg :ray_origin
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (ray_direction
    :reader ray_direction
    :initarg :ray_direction
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass ObjectDetection (<ObjectDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autonomous_manipulator-msg:<ObjectDetection> is deprecated: use autonomous_manipulator-msg:ObjectDetection instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:color-val is deprecated.  Use autonomous_manipulator-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:u-val is deprecated.  Use autonomous_manipulator-msg:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:v-val is deprecated.  Use autonomous_manipulator-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'area_px-val :lambda-list '(m))
(cl:defmethod area_px-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:area_px-val is deprecated.  Use autonomous_manipulator-msg:area_px instead.")
  (area_px m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:position-val is deprecated.  Use autonomous_manipulator-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'ray_origin-val :lambda-list '(m))
(cl:defmethod ray_origin-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:ray_origin-val is deprecated.  Use autonomous_manipulator-msg:ray_origin instead.")
  (ray_origin m))

(cl:ensure-generic-function 'ray_direction-val :lambda-list '(m))
(cl:defmethod ray_direction-val ((m <ObjectDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:ray_direction-val is deprecated.  Use autonomous_manipulator-msg:ray_direction instead.")
  (ray_direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectDetection>) ostream)
  "Serializes a message object of type '<ObjectDetection>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ray_origin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ray_direction) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectDetection>) istream)
  "Deserializes a message object of type '<ObjectDetection>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area_px) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ray_origin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ray_direction) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectDetection>)))
  "Returns string type for a message object of type '<ObjectDetection>"
  "autonomous_manipulator/ObjectDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectDetection)))
  "Returns string type for a message object of type 'ObjectDetection"
  "autonomous_manipulator/ObjectDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectDetection>)))
  "Returns md5sum for a message object of type '<ObjectDetection>"
  "cb50bd15ebca29cc2431eb4b3677e8aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectDetection)))
  "Returns md5sum for a message object of type 'ObjectDetection"
  "cb50bd15ebca29cc2431eb4b3677e8aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectDetection>)))
  "Returns full string definition for message of type '<ObjectDetection>"
  (cl:format cl:nil "# One colour blob detected by the perception node.~%string color                       # orange | green | white | yellow~%float32 u                          # image coordinates of the (top-face) centroid [px]~%float32 v~%float32 area_px                    # blob area [px]~%geometry_msgs/Point position       # object position on the ground-plane assumption (plane z = plane_z of the array)~%geometry_msgs/Point ray_origin     # camera centre in the world frame~%geometry_msgs/Vector3 ray_direction  # unit ray through the centroid, world frame~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectDetection)))
  "Returns full string definition for message of type 'ObjectDetection"
  (cl:format cl:nil "# One colour blob detected by the perception node.~%string color                       # orange | green | white | yellow~%float32 u                          # image coordinates of the (top-face) centroid [px]~%float32 v~%float32 area_px                    # blob area [px]~%geometry_msgs/Point position       # object position on the ground-plane assumption (plane z = plane_z of the array)~%geometry_msgs/Point ray_origin     # camera centre in the world frame~%geometry_msgs/Vector3 ray_direction  # unit ray through the centroid, world frame~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectDetection>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'color))
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ray_origin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ray_direction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectDetection
    (cl:cons ':color (color msg))
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
    (cl:cons ':area_px (area_px msg))
    (cl:cons ':position (position msg))
    (cl:cons ':ray_origin (ray_origin msg))
    (cl:cons ':ray_direction (ray_direction msg))
))
