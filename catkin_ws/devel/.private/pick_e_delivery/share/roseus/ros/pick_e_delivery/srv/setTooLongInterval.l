;; Auto-generated. Do not edit!


(when (boundp 'pick_e_delivery::setTooLongInterval)
  (if (not (find-package "PICK_E_DELIVERY"))
    (make-package "PICK_E_DELIVERY"))
  (shadow 'setTooLongInterval (find-package "PICK_E_DELIVERY")))
(unless (find-package "PICK_E_DELIVERY::SETTOOLONGINTERVAL")
  (make-package "PICK_E_DELIVERY::SETTOOLONGINTERVAL"))
(unless (find-package "PICK_E_DELIVERY::SETTOOLONGINTERVALREQUEST")
  (make-package "PICK_E_DELIVERY::SETTOOLONGINTERVALREQUEST"))
(unless (find-package "PICK_E_DELIVERY::SETTOOLONGINTERVALRESPONSE")
  (make-package "PICK_E_DELIVERY::SETTOOLONGINTERVALRESPONSE"))

(in-package "ROS")





(defclass pick_e_delivery::setTooLongIntervalRequest
  :super ros::object
  :slots (_period ))

(defmethod pick_e_delivery::setTooLongIntervalRequest
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

(defclass pick_e_delivery::setTooLongIntervalResponse
  :super ros::object
  :slots (_period ))

(defmethod pick_e_delivery::setTooLongIntervalResponse
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

(defclass pick_e_delivery::setTooLongInterval
  :super ros::object
  :slots ())

(setf (get pick_e_delivery::setTooLongInterval :md5sum-) "32162b1483389f970edd3323d63a7978")
(setf (get pick_e_delivery::setTooLongInterval :datatype-) "pick_e_delivery/setTooLongInterval")
(setf (get pick_e_delivery::setTooLongInterval :request) pick_e_delivery::setTooLongIntervalRequest)
(setf (get pick_e_delivery::setTooLongInterval :response) pick_e_delivery::setTooLongIntervalResponse)

(defmethod pick_e_delivery::setTooLongIntervalRequest
  (:response () (instance pick_e_delivery::setTooLongIntervalResponse :init)))

(setf (get pick_e_delivery::setTooLongIntervalRequest :md5sum-) "32162b1483389f970edd3323d63a7978")
(setf (get pick_e_delivery::setTooLongIntervalRequest :datatype-) "pick_e_delivery/setTooLongIntervalRequest")
(setf (get pick_e_delivery::setTooLongIntervalRequest :definition-)
      "float32 period
---
float32 period
")

(setf (get pick_e_delivery::setTooLongIntervalResponse :md5sum-) "32162b1483389f970edd3323d63a7978")
(setf (get pick_e_delivery::setTooLongIntervalResponse :datatype-) "pick_e_delivery/setTooLongIntervalResponse")
(setf (get pick_e_delivery::setTooLongIntervalResponse :definition-)
      "float32 period
---
float32 period
")



(provide :pick_e_delivery/setTooLongInterval "32162b1483389f970edd3323d63a7978")


