; Auto-generated. Do not edit!


(cl:in-package srrg2_core_ros-msg)


;//! \htmlinclude PlannerStatusMessage.msg.html

(cl:defclass <PlannerStatusMessage> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform "")
   (a_star_status
    :reader a_star_status
    :initarg :a_star_status
    :type cl:integer
    :initform 0)
   (path_type
    :reader path_type
    :initarg :path_type
    :type cl:integer
    :initform 0)
   (cost_to_global_goal
    :reader cost_to_global_goal
    :initarg :cost_to_global_goal
    :type cl:float
    :initform 0.0)
   (distance_to_global_goal
    :reader distance_to_global_goal
    :initarg :distance_to_global_goal
    :type cl:float
    :initform 0.0)
   (distance_to_local_goal
    :reader distance_to_local_goal
    :initarg :distance_to_local_goal
    :type cl:float
    :initform 0.0))
)

(cl:defclass PlannerStatusMessage (<PlannerStatusMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlannerStatusMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlannerStatusMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srrg2_core_ros-msg:<PlannerStatusMessage> is deprecated: use srrg2_core_ros-msg:PlannerStatusMessage instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PlannerStatusMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:header-val is deprecated.  Use srrg2_core_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <PlannerStatusMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:status-val is deprecated.  Use srrg2_core_ros-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'a_star_status-val :lambda-list '(m))
(cl:defmethod a_star_status-val ((m <PlannerStatusMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:a_star_status-val is deprecated.  Use srrg2_core_ros-msg:a_star_status instead.")
  (a_star_status m))

(cl:ensure-generic-function 'path_type-val :lambda-list '(m))
(cl:defmethod path_type-val ((m <PlannerStatusMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:path_type-val is deprecated.  Use srrg2_core_ros-msg:path_type instead.")
  (path_type m))

(cl:ensure-generic-function 'cost_to_global_goal-val :lambda-list '(m))
(cl:defmethod cost_to_global_goal-val ((m <PlannerStatusMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:cost_to_global_goal-val is deprecated.  Use srrg2_core_ros-msg:cost_to_global_goal instead.")
  (cost_to_global_goal m))

(cl:ensure-generic-function 'distance_to_global_goal-val :lambda-list '(m))
(cl:defmethod distance_to_global_goal-val ((m <PlannerStatusMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:distance_to_global_goal-val is deprecated.  Use srrg2_core_ros-msg:distance_to_global_goal instead.")
  (distance_to_global_goal m))

(cl:ensure-generic-function 'distance_to_local_goal-val :lambda-list '(m))
(cl:defmethod distance_to_local_goal-val ((m <PlannerStatusMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_core_ros-msg:distance_to_local_goal-val is deprecated.  Use srrg2_core_ros-msg:distance_to_local_goal instead.")
  (distance_to_local_goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlannerStatusMessage>) ostream)
  "Serializes a message object of type '<PlannerStatusMessage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
  (cl:let* ((signed (cl:slot-value msg 'a_star_status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'path_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cost_to_global_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_to_global_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_to_local_goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlannerStatusMessage>) istream)
  "Deserializes a message object of type '<PlannerStatusMessage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a_star_status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path_type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cost_to_global_goal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_global_goal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_to_local_goal) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlannerStatusMessage>)))
  "Returns string type for a message object of type '<PlannerStatusMessage>"
  "srrg2_core_ros/PlannerStatusMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlannerStatusMessage)))
  "Returns string type for a message object of type 'PlannerStatusMessage"
  "srrg2_core_ros/PlannerStatusMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlannerStatusMessage>)))
  "Returns md5sum for a message object of type '<PlannerStatusMessage>"
  "cde3a14282df6b919fe6c0fcba349400")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlannerStatusMessage)))
  "Returns md5sum for a message object of type 'PlannerStatusMessage"
  "cde3a14282df6b919fe6c0fcba349400")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlannerStatusMessage>)))
  "Returns full string definition for message of type '<PlannerStatusMessage>"
  (cl:format cl:nil "#gg mesage for the planner status, shot continuosly by the planner~%~%Header   header~%string   status           #status of the planner (\"idle\", \"moving\", \"invalid_goal\", \"unreachable\")~%int32    a_star_status    #result of the enum for the local path search~%int32    path_type        #type of path 0: gradient, 1: grid~%float32  cost_to_global_goal     #value of the cost function at the current location~%float32  distance_to_global_goal #distance to the current location [meters]~%float32  distance_to_local_goal #distance to the current location [meters]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlannerStatusMessage)))
  "Returns full string definition for message of type 'PlannerStatusMessage"
  (cl:format cl:nil "#gg mesage for the planner status, shot continuosly by the planner~%~%Header   header~%string   status           #status of the planner (\"idle\", \"moving\", \"invalid_goal\", \"unreachable\")~%int32    a_star_status    #result of the enum for the local path search~%int32    path_type        #type of path 0: gradient, 1: grid~%float32  cost_to_global_goal     #value of the cost function at the current location~%float32  distance_to_global_goal #distance to the current location [meters]~%float32  distance_to_local_goal #distance to the current location [meters]~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlannerStatusMessage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'status))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlannerStatusMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'PlannerStatusMessage
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
    (cl:cons ':a_star_status (a_star_status msg))
    (cl:cons ':path_type (path_type msg))
    (cl:cons ':cost_to_global_goal (cost_to_global_goal msg))
    (cl:cons ':distance_to_global_goal (distance_to_global_goal msg))
    (cl:cons ':distance_to_local_goal (distance_to_local_goal msg))
))
