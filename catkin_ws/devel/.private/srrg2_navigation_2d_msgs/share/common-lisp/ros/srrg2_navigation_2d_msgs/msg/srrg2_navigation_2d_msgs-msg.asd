
(cl:in-package :asdf)

(defsystem "srrg2_navigation_2d_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CollisionAvoiderStatus" :depends-on ("_package_CollisionAvoiderStatus"))
    (:file "_package_CollisionAvoiderStatus" :depends-on ("_package"))
    (:file "LocalPathPlannerStatus" :depends-on ("_package_LocalPathPlannerStatus"))
    (:file "_package_LocalPathPlannerStatus" :depends-on ("_package"))
    (:file "PathFollowerStatus" :depends-on ("_package_PathFollowerStatus"))
    (:file "_package_PathFollowerStatus" :depends-on ("_package"))
  ))