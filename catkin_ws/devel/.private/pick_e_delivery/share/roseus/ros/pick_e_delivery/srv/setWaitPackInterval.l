;; Auto-generated. Do not edit!


(when (boundp 'pick_e_delivery::setWaitPackInterval)
  (if (not (find-package "PICK_E_DELIVERY"))
    (make-package "PICK_E_DELIVERY"))
  (shadow 'setWaitPackInterval (find-package "PICK_E_DELIVERY")))
(unless (find-package "PICK_E_DELIVERY::SETWAITPACKINTERVAL")
  (make-package "PICK_E_DELIVERY::SETWAITPACKINTERVAL"))
(unless (find-package "PICK_E_DELIVERY::SETWAITPACKINTERVALREQUEST")
  (make-package "PICK_E_DELIVERY::SETWAITPACKINTERVALREQUEST"))
(unless (find-package "PICK_E_DELIVERY::SETWAITPACKINTERVALRESPONSE")
  (make-package "PICK_E_DELIVERY::SETWAITPACKINTERVALRESPONSE"))

(in-package "ROS")





(defclass pick_e_delivery::setWaitPackIntervalRequest
  :super ros::object
  :slots (_period ))

(defmethod pick_e_delivery::setWaitPackIntervalRequest
  (:init
   (&key
    ((:period __period) 0.0)
    )
   (send-super :init)
   (setq _period (float __period))
   self)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:serialization-length
   ()
   (+
    ;; float32 _period
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _period
       (sys::poke _period (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _period
     (setq _period (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pick_e_delivery::setWaitPackIntervalResponse
  :super ros::object
  :slots (_period ))

(defmethod pick_e_delivery::setWaitPackIntervalResponse
  (:init
   (&key
    ((:period __period) 0.0)
    )
   (send-super :init)
   (setq _period (float __period))
   self)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:serialization-length
   ()
   (+
    ;; float32 _period
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _period
       (sys::poke _period (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _period
     (setq _period (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pick_e_delivery::setWaitPackInterval
  :super ros::object
  :slots ())

(setf (get pick_e_delivery::setWaitPackInterval :md5sum-) "32162b1483389f970edd3323d63a7978")
(setf (get pick_e_delivery::setWaitPackInterval :datatype-) "pick_e_delivery/setWaitPackInterval")
(setf (get pick_e_delivery::setWaitPackInterval :request) pick_e_delivery::setWaitPackIntervalRequest)
(setf (get pick_e_delivery::setWaitPackInterval :response) pick_e_delivery::setWaitPackIntervalResponse)

(defmethod pick_e_delivery::setWaitPackIntervalRequest
  (:response () (instance pick_e_delivery::setWaitPackIntervalResponse :init)))

(setf (get pick_e_delivery::setWaitPackIntervalRequest :md5sum-) "32162b1483389f970edd3323d63a7978")
(setf (get pick_e_delivery::setWaitPackIntervalRequest :datatype-) "pick_e_delivery/setWaitPackIntervalRequest")
(setf (get pick_e_delivery::setWaitPackIntervalRequest :definition-)
      "float32 period
---
float32 period
")

(setf (get pick_e_delivery::setWaitPackIntervalResponse :md5sum-) "32162b1483389f970edd3323d63a7978")
(setf (get pick_e_delivery::setWaitPackIntervalResponse :datatype-) "pick_e_delivery/setWaitPackIntervalResponse")
(setf (get pick_e_delivery::setWaitPackIntervalResponse :definition-)
      "float32 period
---
float32 period
")



(provide :pick_e_delivery/setWaitPackInterval "32162b1483389f970edd3323d63a7978")


