;; Auto-generated. Do not edit!


(when (boundp 'autonomous_manipulator::ObjectDetection)
  (if (not (find-package "AUTONOMOUS_MANIPULATOR"))
    (make-package "AUTONOMOUS_MANIPULATOR"))
  (shadow 'ObjectDetection (find-package "AUTONOMOUS_MANIPULATOR")))
(unless (find-package "AUTONOMOUS_MANIPULATOR::OBJECTDETECTION")
  (make-package "AUTONOMOUS_MANIPULATOR::OBJECTDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ObjectDetection.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass autonomous_manipulator::ObjectDetection
  :super ros::object
  :slots (_color _u _v _area_px _position _ray_origin _ray_direction ))

(defmethod autonomous_manipulator::ObjectDetection
  (:init
   (&key
    ((:color __color) "")
    ((:u __u) 0.0)
    ((:v __v) 0.0)
    ((:area_px __area_px) 0.0)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:ray_origin __ray_origin) (instance geometry_msgs::Point :init))
    ((:ray_direction __ray_direction) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _color (string __color))
   (setq _u (float __u))
   (setq _v (float __v))
   (setq _area_px (float __area_px))
   (setq _position __position)
   (setq _ray_origin __ray_origin)
   (setq _ray_direction __ray_direction)
   self)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:area_px
   (&optional __area_px)
   (if __area_px (setq _area_px __area_px)) _area_px)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:ray_origin
   (&rest __ray_origin)
   (if (keywordp (car __ray_origin))
       (send* _ray_origin __ray_origin)
     (progn
       (if __ray_origin (setq _ray_origin (car __ray_origin)))
       _ray_origin)))
  (:ray_direction
   (&rest __ray_direction)
   (if (keywordp (car __ray_direction))
       (send* _ray_direction __ray_direction)
     (progn
       (if __ray_direction (setq _ray_direction (car __ray_direction)))
       _ray_direction)))
  (:serialization-length
   ()
   (+
    ;; string _color
    4 (length _color)
    ;; float32 _u
    4
    ;; float32 _v
    4
    ;; float32 _area_px
    4
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Point _ray_origin
    (send _ray_origin :serialization-length)
    ;; geometry_msgs/Vector3 _ray_direction
    (send _ray_direction :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;; float32 _u
       (sys::poke _u (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v
       (sys::poke _v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _area_px
       (sys::poke _area_px (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Point _ray_origin
       (send _ray_origin :serialize s)
     ;; geometry_msgs/Vector3 _ray_direction
       (send _ray_direction :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _u
     (setq _u (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v
     (setq _v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _area_px
     (setq _area_px (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Point _ray_origin
     (send _ray_origin :deserialize buf ptr-) (incf ptr- (send _ray_origin :serialization-length))
   ;; geometry_msgs/Vector3 _ray_direction
     (send _ray_direction :deserialize buf ptr-) (incf ptr- (send _ray_direction :serialization-length))
   ;;
   self)
  )

(setf (get autonomous_manipulator::ObjectDetection :md5sum-) "cb50bd15ebca29cc2431eb4b3677e8aa")
(setf (get autonomous_manipulator::ObjectDetection :datatype-) "autonomous_manipulator/ObjectDetection")
(setf (get autonomous_manipulator::ObjectDetection :definition-)
      "# One colour blob detected by the perception node.
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



(provide :autonomous_manipulator/ObjectDetection "cb50bd15ebca29cc2431eb4b3677e8aa")


