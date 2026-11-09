;; Auto-generated. Do not edit!


(when (boundp 'autonomous_manipulator::ObjectDetectionArray)
  (if (not (find-package "AUTONOMOUS_MANIPULATOR"))
    (make-package "AUTONOMOUS_MANIPULATOR"))
  (shadow 'ObjectDetectionArray (find-package "AUTONOMOUS_MANIPULATOR")))
(unless (find-package "AUTONOMOUS_MANIPULATOR::OBJECTDETECTIONARRAY")
  (make-package "AUTONOMOUS_MANIPULATOR::OBJECTDETECTIONARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjectDetectionArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass autonomous_manipulator::ObjectDetectionArray
  :super ros::object
  :slots (_header _plane_z _detections ))

(defmethod autonomous_manipulator::ObjectDetectionArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:plane_z __plane_z) 0.0)
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _plane_z (float __plane_z))
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:plane_z
   (&optional __plane_z)
   (if __plane_z (setq _plane_z __plane_z)) _plane_z)
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _plane_z
    4
    ;; autonomous_manipulator/ObjectDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _plane_z
       (sys::poke _plane_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; autonomous_manipulator/ObjectDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _plane_z
     (setq _plane_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; autonomous_manipulator/ObjectDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance autonomous_manipulator::ObjectDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get autonomous_manipulator::ObjectDetectionArray :md5sum-) "d115234f9c783e5b7bad924fd24b51dd")
(setf (get autonomous_manipulator::ObjectDetectionArray :datatype-) "autonomous_manipulator/ObjectDetectionArray")
(setf (get autonomous_manipulator::ObjectDetectionArray :definition-)
      "std_msgs/Header header             # stamp = image stamp (simulation time)
float32 plane_z                    # height of the plane used to compute ObjectDetection.position
ObjectDetection[] detections

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

================================================================================
MSG: autonomous_manipulator/ObjectDetection
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
")



(provide :autonomous_manipulator/ObjectDetectionArray "d115234f9c783e5b7bad924fd24b51dd")


