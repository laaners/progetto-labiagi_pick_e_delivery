; Auto-generated. Do not edit!


(cl:in-package srrg2_navigation_2d_msgs-msg)


;//! \htmlinclude CollisionAvoiderStatus.msg.html

(cl:defclass <CollisionAvoiderStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cmd_vel_input
    :reader cmd_vel_input
    :initarg :cmd_vel_input
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (cmd_vel_output
    :reader cmd_vel_output
    :initarg :cmd_vel_output
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass CollisionAvoiderStatus (<CollisionAvoiderStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CollisionAvoiderStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CollisionAvoiderStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srrg2_navigation_2d_msgs-msg:<CollisionAvoiderStatus> is deprecated: use srrg2_navigation_2d_msgs-msg:CollisionAvoiderStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CollisionAvoiderStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:header-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cmd_vel_input-val :lambda-list '(m))
(cl:defmethod cmd_vel_input-val ((m <CollisionAvoiderStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:cmd_vel_input-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:cmd_vel_input instead.")
  (cmd_vel_input m))

(cl:ensure-generic-function 'cmd_vel_output-val :lambda-list '(m))
(cl:defmethod cmd_vel_output-val ((m <CollisionAvoiderStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:cmd_vel_output-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:cmd_vel_output instead.")
  (cmd_vel_output m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <CollisionAvoiderStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:status-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CollisionAvoiderStatus>) ostream)
  "Serializes a message object of type '<CollisionAvoiderStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd_vel_input) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd_vel_output) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CollisionAvoiderStatus>) istream)
  "Deserializes a message object of type '<CollisionAvoiderStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd_vel_input) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd_vel_output) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CollisionAvoiderStatus>)))
  "Returns string type for a message object of type '<CollisionAvoiderStatus>"
  "srrg2_navigation_2d_msgs/CollisionAvoiderStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CollisionAvoiderStatus)))
  "Returns string type for a message object of type 'CollisionAvoiderStatus"
  "srrg2_navigation_2d_msgs/CollisionAvoiderStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CollisionAvoiderStatus>)))
  "Returns md5sum for a message object of type '<CollisionAvoiderStatus>"
  "413ea057b6949dd5b85094f0459afba5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CollisionAvoiderStatus)))
  "Returns md5sum for a message object of type 'CollisionAvoiderStatus"
  "413ea057b6949dd5b85094f0459afba5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CollisionAvoiderStatus>)))
  "Returns full string definition for message of type '<CollisionAvoiderStatus>"
  (cl:format cl:nil "Header header~%~%#command input to the collision avoider~%geometry_msgs/Twist cmd_vel_input~%~%#command output by the collision avoider~%geometry_msgs/Twist cmd_vel_output~%~%~%# status can be either~%# \"clear\":    when the obstacles are fare enough and the avoider does not kick in~%# \"adjusting\": when the obstacles affect the control, but don't block the robot~%# \"blocked\":   when the robot stopped due to a dynamic obstacle~%~%string status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CollisionAvoiderStatus)))
  "Returns full string definition for message of type 'CollisionAvoiderStatus"
  (cl:format cl:nil "Header header~%~%#command input to the collision avoider~%geometry_msgs/Twist cmd_vel_input~%~%#command output by the collision avoider~%geometry_msgs/Twist cmd_vel_output~%~%~%# status can be either~%# \"clear\":    when the obstacles are fare enough and the avoider does not kick in~%# \"adjusting\": when the obstacles affect the control, but don't block the robot~%# \"blocked\":   when the robot stopped due to a dynamic obstacle~%~%string status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CollisionAvoiderStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd_vel_input))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd_vel_output))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CollisionAvoiderStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'CollisionAvoiderStatus
    (cl:cons ':header (header msg))
    (cl:cons ':cmd_vel_input (cmd_vel_input msg))
    (cl:cons ':cmd_vel_output (cmd_vel_output msg))
    (cl:cons ':status (status msg))
))
