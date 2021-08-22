
(cl:in-package :asdf)

(defsystem "pick_e_delivery-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NewGoal" :depends-on ("_package_NewGoal"))
    (:file "_package_NewGoal" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Timeout" :depends-on ("_package_Timeout"))
    (:file "_package_Timeout" :depends-on ("_package"))
  ))