; Auto-generated. Do not edit!


(cl:in-package srrg2_core_ros-msg)


;//! \htmlinclude ViewerBufferMessage.msg.html

(cl:defclass <ViewerBufferMessage> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (num_packets
    :reader num_packets
    :initarg :num_packets
    :type cl:integer
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ViewerBufferMessage (<ViewerBufferMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ViewerBufferMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ViewerBufferMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srrg2_core_ros-msg:<ViewerBufferMessage> is deprecated: use srrg2_core_ros-msg:ViewerBufferMessage instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ViewerBufferMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:header-val is deprecated.  Use srrg2_core_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <ViewerBufferMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:size-val is deprecated.  Use srrg2_core_ros-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'num_packets-val :lambda-list '(m))
(cl:defmethod num_packets-val ((m <ViewerBufferMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:num_packets-val is deprecated.  Use srrg2_core_ros-msg:num_packets instead.")
  (num_packets m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ViewerBufferMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:status-val is deprecated.  Use srrg2_core_ros-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ViewerBufferMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:data-val is deprecated.  Use srrg2_core_ros-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ViewerBufferMessage>) ostream)
  "Serializes a message object of type '<ViewerBufferMessage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'num_packets)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ViewerBufferMessage>) istream)
  "Deserializes a message object of type '<ViewerBufferMessage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'num_packets)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ViewerBufferMessage>)))
  "Returns string type for a message object of type '<ViewerBufferMessage>"
  "srrg2_core_ros/ViewerBufferMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ViewerBufferMessage)))
  "Returns string type for a message object of type 'ViewerBufferMessage"
  "srrg2_core_ros/ViewerBufferMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ViewerBufferMessage>)))
  "Returns md5sum for a message object of type '<ViewerBufferMessage>"
  "add7e000fddfec4b3922d4f8c99195b3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ViewerBufferMessage)))
  "Returns md5sum for a message object of type 'ViewerBufferMessage"
  "add7e000fddfec4b3922d4f8c99195b3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ViewerBufferMessage>)))
  "Returns full string definition for message of type '<ViewerBufferMessage>"
  (cl:format cl:nil "#ia generic viewer buffer message. it contains all the fields of a MemoryBuffer~%#ia in this way we use ros to send serialized packets containing objects and commands~%#ia those can be either:~%#ia   - received, deserialized and passed through a qglVieweport~%#ia   - received, deserialized and converted into RViz compliant stuff~%~%Header  header~%uint64 size           #ia size in bytes~%uint64 num_packets    #ia number of packets serialized in this buffer~%uint8 status          #ia status of the buffer (Free=0, Read=1, Write=2, Ready=3)~%uint8[] data          #ia actual buffer of stuff~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ViewerBufferMessage)))
  "Returns full string definition for message of type 'ViewerBufferMessage"
  (cl:format cl:nil "#ia generic viewer buffer message. it contains all the fields of a MemoryBuffer~%#ia in this way we use ros to send serialized packets containing objects and commands~%#ia those can be either:~%#ia   - received, deserialized and passed through a qglVieweport~%#ia   - received, deserialized and converted into RViz compliant stuff~%~%Header  header~%uint64 size           #ia size in bytes~%uint64 num_packets    #ia number of packets serialized in this buffer~%uint8 status          #ia status of the buffer (Free=0, Read=1, Write=2, Ready=3)~%uint8[] data          #ia actual buffer of stuff~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ViewerBufferMessage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ViewerBufferMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'ViewerBufferMessage
    (cl:cons ':header (header msg))
    (cl:cons ':size (size msg))
    (cl:cons ':num_packets (num_packets msg))
    (cl:cons ':status (status msg))
    (cl:cons ':data (data msg))
))
