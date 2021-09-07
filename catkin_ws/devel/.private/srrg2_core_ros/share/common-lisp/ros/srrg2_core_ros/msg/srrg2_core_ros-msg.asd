
(cl:in-package :asdf)

(defsystem "srrg2_core_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PlannerStatusMessage" :depends-on ("_package_PlannerStatusMessage"))
    (:file "_package_PlannerStatusMessage" :depends-on ("_package"))
    (:file "ViewerBufferMessage" :depends-on ("_package_ViewerBufferMessage"))
    (:file "_package_ViewerBufferMessage" :depends-on ("_package"))
  ))