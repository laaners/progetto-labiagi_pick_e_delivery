; Auto-generated. Do not edit!


(cl:in-package pick_e_delivery-msg)


;//! \htmlinclude Timeout.msg.html

(cl:defclass <Timeout> (roslisp-msg-protocol:ros-message)
  ((event
    :reader event
    :initarg :event
    :type cl:string
    :initform ""))
)

(cl:defclass Timeout (<Timeout>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Timeout>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Timeout)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_e_delivery-msg:<Timeout> is deprecated: use pick_e_delivery-msg:Timeout instead.")))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <Timeout>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_e_delivery-msg:event-val is deprecated.  Use pick_e_delivery-msg:event instead.")
  (event m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Timeout>) ostream)
  "Serializes a message object of type '<Timeout>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'event))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Timeout>) istream)
  "Deserializes a message object of type '<Timeout>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'event) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'event) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Timeout>)))
  "Returns string type for a message object of type '<Timeout>"
  "pick_e_delivery/Timeout")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Timeout)))
  "Returns string type for a message object of type 'Timeout"
  "pick_e_delivery/Timeout")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Timeout>)))
  "Returns md5sum for a message object of type '<Timeout>"
  "6aea470b1ed54075e83032ee4be16538")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Timeout)))
  "Returns md5sum for a message object of type 'Timeout"
  "6aea470b1ed54075e83032ee4be16538")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Timeout>)))
  "Returns full string definition for message of type '<Timeout>"
  (cl:format cl:nil "string event~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Timeout)))
  "Returns full string definition for message of type 'Timeout"
  (cl:format cl:nil "string event~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Timeout>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'event))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Timeout>))
  "Converts a ROS message object to a list"
  (cl:list 'Timeout
    (cl:cons ':event (event msg))
))
