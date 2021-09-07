;; Auto-generated. Do not edit!


(when (boundp 'srrg2_core_ros::ViewerBufferMessage)
  (if (not (find-package "SRRG2_CORE_ROS"))
    (make-package "SRRG2_CORE_ROS"))
  (shadow 'ViewerBufferMessage (find-package "SRRG2_CORE_ROS")))
(unless (find-package "SRRG2_CORE_ROS::VIEWERBUFFERMESSAGE")
  (make-package "SRRG2_CORE_ROS::VIEWERBUFFERMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude ViewerBufferMessage.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass srrg2_core_ros::ViewerBufferMessage
  :super ros::object
  :slots (_header _size _num_packets _status _data ))

(defmethod srrg2_core_ros::ViewerBufferMessage
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:size __size) 0)
    ((:num_packets __num_packets) 0)
    ((:status __status) 0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _size (round __size))
   (setq _num_packets (round __num_packets))
   (setq _status (round __status))
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:num_packets
   (&optional __num_packets)
   (if __num_packets (setq _num_packets __num_packets)) _num_packets)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _size
    8
    ;; uint64 _num_packets
    8
    ;; uint8 _status
    1
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _size
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _size (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _size) (= (length (_size . bv)) 2)) ;; bignum
              (write-long (ash (elt (_size . bv) 0) 0) s)
              (write-long (ash (elt (_size . bv) 1) -1) s))
             ((and (class _size) (= (length (_size . bv)) 1)) ;; big1
              (write-long (elt (_size . bv) 0) s)
              (write-long (if (>= _size 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _size s)(write-long (if (>= _size 0) 0 #xffffffff) s)))
     ;; uint64 _num_packets
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_packets (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_packets) (= (length (_num_packets . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_packets . bv) 0) 0) s)
              (write-long (ash (elt (_num_packets . bv) 1) -1) s))
             ((and (class _num_packets) (= (length (_num_packets . bv)) 1)) ;; big1
              (write-long (elt (_num_packets . bv) 0) s)
              (write-long (if (>= _num_packets 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_packets s)(write-long (if (>= _num_packets 0) 0 #xffffffff) s)))
     ;; uint8 _status
       (write-byte _status s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _size
     
#+(or :alpha :irix6 :x86_64)
      (setf _size (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _size (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint64 _num_packets
     
#+(or :alpha :irix6 :x86_64)
      (setf _num_packets (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_packets (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get srrg2_core_ros::ViewerBufferMessage :md5sum-) "add7e000fddfec4b3922d4f8c99195b3")
(setf (get srrg2_core_ros::ViewerBufferMessage :datatype-) "srrg2_core_ros/ViewerBufferMessage")
(setf (get srrg2_core_ros::ViewerBufferMessage :definition-)
      "#ia generic viewer buffer message. it contains all the fields of a MemoryBuffer
#ia in this way we use ros to send serialized packets containing objects and commands
#ia those can be either:
#ia   - received, deserialized and passed through a qglVieweport
#ia   - received, deserialized and converted into RViz compliant stuff

Header  header
uint64 size           #ia size in bytes
uint64 num_packets    #ia number of packets serialized in this buffer
uint8 status          #ia status of the buffer (Free=0, Read=1, Write=2, Ready=3)
uint8[] data          #ia actual buffer of stuff

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :srrg2_core_ros/ViewerBufferMessage "add7e000fddfec4b3922d4f8c99195b3")


