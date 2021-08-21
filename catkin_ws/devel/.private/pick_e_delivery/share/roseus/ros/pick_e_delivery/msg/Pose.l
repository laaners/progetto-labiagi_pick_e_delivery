;; Auto-generated. Do not edit!


(when (boundp 'pick_e_delivery::Pose)
  (if (not (find-package "PICK_E_DELIVERY"))
    (make-package "PICK_E_DELIVERY"))
  (shadow 'Pose (find-package "PICK_E_DELIVERY")))
(unless (find-package "PICK_E_DELIVERY::POSE")
  (make-package "PICK_E_DELIVERY::POSE"))

(in-package "ROS")
;;//! \htmlinclude Pose.msg.html


(defclass pick_e_delivery::Pose
  :super ros::object
  :slots (_x _y _yaw _status _status_msg _sender _receiver ))

(defmethod pick_e_delivery::Pose
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:status __status) 0)
    ((:status_msg __status_msg) "")
    ((:sender __sender) "")
    ((:receiver __receiver) "")
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   (setq _status (round __status))
   (setq _status_msg (string __status_msg))
   (setq _sender (string __sender))
   (setq _receiver (string __receiver))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:status_msg
   (&optional __status_msg)
   (if __status_msg (setq _status_msg __status_msg)) _status_msg)
  (:sender
   (&optional __sender)
   (if __sender (setq _sender __sender)) _sender)
  (:receiver
   (&optional __receiver)
   (if __receiver (setq _receiver __receiver)) _receiver)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _yaw
    4
    ;; int32 _status
    4
    ;; string _status_msg
    4 (length _status_msg)
    ;; string _sender
    4 (length _sender)
    ;; string _receiver
    4 (length _receiver)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _status
       (write-long _status s)
     ;; string _status_msg
       (write-long (length _status_msg) s) (princ _status_msg s)
     ;; string _sender
       (write-long (length _sender) s) (princ _sender s)
     ;; string _receiver
       (write-long (length _receiver) s) (princ _receiver s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _status
     (setq _status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _status_msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _sender
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sender (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _receiver
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _receiver (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pick_e_delivery::Pose :md5sum-) "0281e339f76520d24da4f5c9c77c5df7")
(setf (get pick_e_delivery::Pose :datatype-) "pick_e_delivery/Pose")
(setf (get pick_e_delivery::Pose :definition-)
      "float32 x
float32 y
float32 yaw
int32 status
string status_msg
string sender
string receiver
")



(provide :pick_e_delivery/Pose "0281e339f76520d24da4f5c9c77c5df7")


