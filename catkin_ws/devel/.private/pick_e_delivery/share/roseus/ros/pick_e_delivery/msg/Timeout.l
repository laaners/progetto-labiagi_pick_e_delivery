;; Auto-generated. Do not edit!


(when (boundp 'pick_e_delivery::Timeout)
  (if (not (find-package "PICK_E_DELIVERY"))
    (make-package "PICK_E_DELIVERY"))
  (shadow 'Timeout (find-package "PICK_E_DELIVERY")))
(unless (find-package "PICK_E_DELIVERY::TIMEOUT")
  (make-package "PICK_E_DELIVERY::TIMEOUT"))

(in-package "ROS")
;;//! \htmlinclude Timeout.msg.html


(defclass pick_e_delivery::Timeout
  :super ros::object
  :slots (_event ))

(defmethod pick_e_delivery::Timeout
  (:init
   (&key
    ((:event __event) "")
    )
   (send-super :init)
   (setq _event (string __event))
   self)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:serialization-length
   ()
   (+
    ;; string _event
    4 (length _event)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _event
       (write-long (length _event) s) (princ _event s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _event
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _event (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pick_e_delivery::Timeout :md5sum-) "6aea470b1ed54075e83032ee4be16538")
(setf (get pick_e_delivery::Timeout :datatype-) "pick_e_delivery/Timeout")
(setf (get pick_e_delivery::Timeout :definition-)
      "string event
")



(provide :pick_e_delivery/Timeout "6aea470b1ed54075e83032ee4be16538")


