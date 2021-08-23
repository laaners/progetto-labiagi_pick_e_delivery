; Auto-generated. Do not edit!


(cl:in-package pick_e_delivery-srv)


;//! \htmlinclude setTooLongInterval-request.msg.html

(cl:defclass <setTooLongInterval-request> (roslisp-msg-protocol:ros-message)
  ((period
    :reader period
    :initarg :period
    :type cl:float
    :initform 0.0))
)

(cl:defclass setTooLongInterval-request (<setTooLongInterval-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setTooLongInterval-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setTooLongInterval-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_e_delivery-srv:<setTooLongInterval-request> is deprecated: use pick_e_delivery-srv:setTooLongInterval-request instead.")))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <setTooLongInterval-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_e_delivery-srv:period-val is deprecated.  Use pick_e_delivery-srv:period instead.")
  (period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setTooLongInterval-request>) ostream)
  "Serializes a message object of type '<setTooLongInterval-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setTooLongInterval-request>) istream)
  "Deserializes a message object of type '<setTooLongInterval-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'period) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setTooLongInterval-request>)))
  "Returns string type for a service object of type '<setTooLongInterval-request>"
  "pick_e_delivery/setTooLongIntervalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTooLongInterval-request)))
  "Returns string type for a service object of type 'setTooLongInterval-request"
  "pick_e_delivery/setTooLongIntervalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setTooLongInterval-request>)))
  "Returns md5sum for a message object of type '<setTooLongInterval-request>"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setTooLongInterval-request)))
  "Returns md5sum for a message object of type 'setTooLongInterval-request"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setTooLongInterval-request>)))
  "Returns full string definition for message of type '<setTooLongInterval-request>"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setTooLongInterval-request)))
  "Returns full string definition for message of type 'setTooLongInterval-request"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setTooLongInterval-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setTooLongInterval-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setTooLongInterval-request
    (cl:cons ':period (period msg))
))
;//! \htmlinclude setTooLongInterval-response.msg.html

(cl:defclass <setTooLongInterval-response> (roslisp-msg-protocol:ros-message)
  ((period
    :reader period
    :initarg :period
    :type cl:float
    :initform 0.0))
)

(cl:defclass setTooLongInterval-response (<setTooLongInterval-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setTooLongInterval-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setTooLongInterval-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_e_delivery-srv:<setTooLongInterval-response> is deprecated: use pick_e_delivery-srv:setTooLongInterval-response instead.")))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <setTooLongInterval-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_e_delivery-srv:period-val is deprecated.  Use pick_e_delivery-srv:period instead.")
  (period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setTooLongInterval-response>) ostream)
  "Serializes a message object of type '<setTooLongInterval-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setTooLongInterval-response>) istream)
  "Deserializes a message object of type '<setTooLongInterval-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'period) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setTooLongInterval-response>)))
  "Returns string type for a service object of type '<setTooLongInterval-response>"
  "pick_e_delivery/setTooLongIntervalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTooLongInterval-response)))
  "Returns string type for a service object of type 'setTooLongInterval-response"
  "pick_e_delivery/setTooLongIntervalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setTooLongInterval-response>)))
  "Returns md5sum for a message object of type '<setTooLongInterval-response>"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setTooLongInterval-response)))
  "Returns md5sum for a message object of type 'setTooLongInterval-response"
  "32162b1483389f970edd3323d63a7978")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setTooLongInterval-response>)))
  "Returns full string definition for message of type '<setTooLongInterval-response>"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setTooLongInterval-response)))
  "Returns full string definition for message of type 'setTooLongInterval-response"
  (cl:format cl:nil "float32 period~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setTooLongInterval-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setTooLongInterval-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setTooLongInterval-response
    (cl:cons ':period (period msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setTooLongInterval)))
  'setTooLongInterval-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setTooLongInterval)))
  'setTooLongInterval-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setTooLongInterval)))
  "Returns string type for a service object of type '<setTooLongInterval>"
  "pick_e_delivery/setTooLongInterval")