; Auto-generated. Do not edit!


(cl:in-package pick_e_delivery-srv)


;//! \htmlinclude setWaitPackInterval-request.msg.html

(cl:defclass <setWaitPackInterval-request> (roslisp-msg-protocol:ros-message)
  ((period
    :reader period
    :initarg :period
    :type cl:float
    :initform 0.0))
)

(cl:defclass setWaitPackInterval-request (<setWaitPackInterval-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setWaitPackInterval-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setWaitPackInterval-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_e_delivery-srv:<setWaitPackInterval-request> is deprecated: use pick_e_delivery-srv:setWaitPackInterval-request instead.")))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <setWaitPackInterval-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_e_delivery-srv:period-val is deprecated.  Use pick_e_delivery-srv:period instead.")
  (period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setWaitPackInterval-request>) ostream)
  "Serializes a message object of type '<setWaitPackInterval-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setWaitPackInterval-request>) istream)
  "Deserializes a message object of type '<setWaitPackInterval-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'period) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setWaitPackInterval-request>)))
  "Returns string type for a service object of type '<setWaitPackInterval-request>"
  "pick_e_delivery/setWaitPackIntervalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setWaitPackInterval-request)))
  "Returns string type for a service object of type 'setWaitPackInterval-request"
  "pick_e_delivery/setWaitPackIntervalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setWaitPackInterval-request>)))
  "Returns md5sum for a message object of type '<setWaitPackInterval-request>"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setWaitPackInterval-request)))
  "Returns md5sum for a message object of type 'setWaitPackInterval-request"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setWaitPackInterval-request>)))
  "Returns full string definition for message of type '<setWaitPackInterval-request>"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setWaitPackInterval-request)))
  "Returns full string definition for message of type 'setWaitPackInterval-request"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setWaitPackInterval-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setWaitPackInterval-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setWaitPackInterval-request
    (cl:cons ':period (period msg))
))
;//! \htmlinclude setWaitPackInterval-response.msg.html

(cl:defclass <setWaitPackInterval-response> (roslisp-msg-protocol:ros-message)
  ((period
    :reader period
    :initarg :period
    :type cl:float
    :initform 0.0))
)

(cl:defclass setWaitPackInterval-response (<setWaitPackInterval-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setWaitPackInterval-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setWaitPackInterval-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_e_delivery-srv:<setWaitPackInterval-response> is deprecated: use pick_e_delivery-srv:setWaitPackInterval-response instead.")))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <setWaitPackInterval-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_e_delivery-srv:period-val is deprecated.  Use pick_e_delivery-srv:period instead.")
  (period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setWaitPackInterval-response>) ostream)
  "Serializes a message object of type '<setWaitPackInterval-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setWaitPackInterval-response>) istream)
  "Deserializes a message object of type '<setWaitPackInterval-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'period) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setWaitPackInterval-response>)))
  "Returns string type for a service object of type '<setWaitPackInterval-response>"
  "pick_e_delivery/setWaitPackIntervalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setWaitPackInterval-response)))
  "Returns string type for a service object of type 'setWaitPackInterval-response"
  "pick_e_delivery/setWaitPackIntervalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setWaitPackInterval-response>)))
  "Returns md5sum for a message object of type '<setWaitPackInterval-response>"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setWaitPackInterval-response)))
  "Returns md5sum for a message object of type 'setWaitPackInterval-response"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setWaitPackInterval-response>)))
  "Returns full string definition for message of type '<setWaitPackInterval-response>"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setWaitPackInterval-response)))
  "Returns full string definition for message of type 'setWaitPackInterval-response"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setWaitPackInterval-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setWaitPackInterval-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setWaitPackInterval-response
    (cl:cons ':period (period msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setWaitPackInterval)))
  'setWaitPackInterval-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setWaitPackInterval)))
  'setWaitPackInterval-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setWaitPackInterval)))
  "Returns string type for a service object of type '<setWaitPackInterval>"
  "pick_e_delivery/setWaitPackInterval")