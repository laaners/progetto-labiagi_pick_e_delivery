;; Auto-generated. Do not edit!


(when (boundp 'pick_e_delivery::NewGoal)
  (if (not (find-package "PICK_E_DELIVERY"))
    (make-package "PICK_E_DELIVERY"))
  (shadow 'NewGoal (find-package "PICK_E_DELIVERY")))
(unless (find-package "PICK_E_DELIVERY::NEWGOAL")
  (make-package "PICK_E_DELIVERY::NEWGOAL"))

(in-package "ROS")
;;//! \htmlinclude NewGoal.msg.html


(defclass pick_e_delivery::NewGoal
  :super ros::object
  :slots (_x _y _theta _command _user ))

(defmethod pick_e_delivery::NewGoal
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:theta __theta) 0.0)
    ((:command __command) 0)
    ((:user __user) "")
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _theta (float __theta))
   (setq _command (round __command))
   (setq _user (string __user))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:user
   (&optional __user)
   (if __user (setq _user __user)) _user)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _theta
    4
    ;; int32 _command
    4
    ;; string _user
    4 (length _user)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _command
       (write-long _command s)
     ;; string _user
       (write-long (length _user) s) (princ _user s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _command
     (setq _command (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _user
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _user (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pick_e_delivery::NewGoal :md5sum-) "50266fd61bf27a3712274104f794648f")
(setf (get pick_e_delivery::NewGoal :datatype-) "pick_e_delivery/NewGoal")
(setf (get pick_e_delivery::NewGoal :definition-)
      "float32 x
float32 y
float32 theta
int32 command
string user
")



(provide :pick_e_delivery/NewGoal "50266fd61bf27a3712274104f794648f")


