; Auto-generated. Do not edit!


(cl:in-package autonomous_manipulator-msg)


;//! \htmlinclude ObjectDetectionArray.msg.html

(cl:defclass <ObjectDetectionArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (plane_z
    :reader plane_z
    :initarg :plane_z
    :type cl:float
    :initform 0.0)
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector autonomous_manipulator-msg:ObjectDetection)
   :initform (cl:make-array 0 :element-type 'autonomous_manipulator-msg:ObjectDetection :initial-element (cl:make-instance 'autonomous_manipulator-msg:ObjectDetection))))
)

(cl:defclass ObjectDetectionArray (<ObjectDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autonomous_manipulator-msg:<ObjectDetectionArray> is deprecated: use autonomous_manipulator-msg:ObjectDetectionArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:header-val is deprecated.  Use autonomous_manipulator-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'plane_z-val :lambda-list '(m))
(cl:defmethod plane_z-val ((m <ObjectDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:plane_z-val is deprecated.  Use autonomous_manipulator-msg:plane_z instead.")
  (plane_z m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <ObjectDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autonomous_manipulator-msg:detections-val is deprecated.  Use autonomous_manipulator-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectDetectionArray>) ostream)
  "Serializes a message object of type '<ObjectDetectionArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'plane_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectDetectionArray>) istream)
  "Deserializes a message object of type '<ObjectDetectionArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'plane_z) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'autonomous_manipulator-msg:ObjectDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectDetectionArray>)))
  "Returns string type for a message object of type '<ObjectDetectionArray>"
  "autonomous_manipulator/ObjectDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectDetectionArray)))
  "Returns string type for a message object of type 'ObjectDetectionArray"
  "autonomous_manipulator/ObjectDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectDetectionArray>)))
  "Returns md5sum for a message object of type '<ObjectDetectionArray>"
  "d115234f9c783e5b7bad924fd24b51dd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectDetectionArray)))
  "Returns md5sum for a message object of type 'ObjectDetectionArray"
  "d115234f9c783e5b7bad924fd24b51dd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectDetectionArray>)))
  "Returns full string definition for message of type '<ObjectDetectionArray>"
  (cl:format cl:nil "std_msgs/Header header             # stamp = image stamp (simulation time)~%float32 plane_z                    # height of the plane used to compute ObjectDetection.position~%ObjectDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: autonomous_manipulator/ObjectDetection~%# One colour blob detected by the perception node.~%string color                       # orange | green | white | yellow~%float32 u                          # image coordinates of the (top-face) centroid [px]~%float32 v~%float32 area_px                    # blob area [px]~%geometry_msgs/Point position       # object position on the ground-plane assumption (plane z = plane_z of the array)~%geometry_msgs/Point ray_origin     # camera centre in the world frame~%geometry_msgs/Vector3 ray_direction  # unit ray through the centroid, world frame~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectDetectionArray)))
  "Returns full string definition for message of type 'ObjectDetectionArray"
  (cl:format cl:nil "std_msgs/Header header             # stamp = image stamp (simulation time)~%float32 plane_z                    # height of the plane used to compute ObjectDetection.position~%ObjectDetection[] detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: autonomous_manipulator/ObjectDetection~%# One colour blob detected by the perception node.~%string color                       # orange | green | white | yellow~%float32 u                          # image coordinates of the (top-face) centroid [px]~%float32 v~%float32 area_px                    # blob area [px]~%geometry_msgs/Point position       # object position on the ground-plane assumption (plane z = plane_z of the array)~%geometry_msgs/Point ray_origin     # camera centre in the world frame~%geometry_msgs/Vector3 ray_direction  # unit ray through the centroid, world frame~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectDetectionArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectDetectionArray
    (cl:cons ':header (header msg))
    (cl:cons ':plane_z (plane_z msg))
    (cl:cons ':detections (detections msg))
))
