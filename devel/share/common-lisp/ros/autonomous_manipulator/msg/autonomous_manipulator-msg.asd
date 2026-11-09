
(cl:in-package :asdf)

(defsystem "autonomous_manipulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObjectDetection" :depends-on ("_package_ObjectDetection"))
    (:file "_package_ObjectDetection" :depends-on ("_package"))
    (:file "ObjectDetectionArray" :depends-on ("_package_ObjectDetectionArray"))
    (:file "_package_ObjectDetectionArray" :depends-on ("_package"))
  ))