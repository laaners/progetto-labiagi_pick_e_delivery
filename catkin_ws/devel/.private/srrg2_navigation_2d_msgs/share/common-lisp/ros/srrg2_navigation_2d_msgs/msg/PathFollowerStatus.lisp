; Auto-generated. Do not edit!


(cl:in-package srrg2_navigation_2d_msgs-msg)


;//! \htmlinclude PathFollowerStatus.msg.html

(cl:defclass <PathFollowerStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_steps_to_goal
    :reader num_steps_to_goal
    :initarg :num_steps_to_goal
    :type cl:integer
    :initform 0)
   (robot_pose_2d
    :reader robot_pose_2d
    :initarg :robot_pose_2d
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (local_target_2d
    :reader local_target_2d
    :initarg :local_target_2d
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (cmd_vel
    :reader cmd_vel
    :initarg :cmd_vel
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass PathFollowerStatus (<PathFollowerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathFollowerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathFollowerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name srrg2_navigation_2d_msgs-msg:<PathFollowerStatus> is deprecated: use srrg2_navigation_2d_msgs-msg:PathFollowerStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PathFollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:header-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_steps_to_goal-val :lambda-list '(m))
(cl:defmethod num_steps_to_goal-val ((m <PathFollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:num_steps_to_goal-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:num_steps_to_goal instead.")
  (num_steps_to_goal m))

(cl:ensure-generic-function 'robot_pose_2d-val :lambda-list '(m))
(cl:defmethod robot_pose_2d-val ((m <PathFollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:robot_pose_2d-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:robot_pose_2d instead.")
  (robot_pose_2d m))

(cl:ensure-generic-function 'local_target_2d-val :lambda-list '(m))
(cl:defmethod local_target_2d-val ((m <PathFollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:local_target_2d-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:local_target_2d instead.")
  (local_target_2d m))

(cl:ensure-generic-function 'cmd_vel-val :lambda-list '(m))
(cl:defmethod cmd_vel-val ((m <PathFollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:cmd_vel-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:cmd_vel instead.")
  (cmd_vel m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <PathFollowerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader srrg2_navigation_2d_msgs-msg:status-val is deprecated.  Use srrg2_navigation_2d_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathFollowerStatus>) ostream)
  "Serializes a message object of type '<PathFollowerStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_steps_to_goal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'robot_pose_2d))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'local_target_2d))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd_vel) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathFollowerStatus>) istream)
  "Deserializes a message object of type '<PathFollowerStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_steps_to_goal) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:setf (cl:slot-value msg 'robot_pose_2d) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'robot_pose_2d)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'local_target_2d) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'local_target_2d)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd_vel) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathFollowerStatus>)))
  "Returns string type for a message object of type '<PathFollowerStatus>"
  "srrg2_navigation_2d_msgs/PathFollowerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathFollowerStatus)))
  "Returns string type for a message object of type 'PathFollowerStatus"
  "srrg2_navigation_2d_msgs/PathFollowerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathFollowerStatus>)))
  "Returns md5sum for a message object of type '<PathFollowerStatus>"
  "ea28f7c3c138ef8732745d1ce16b8aaf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathFollowerStatus)))
  "Returns md5sum for a message object of type 'PathFollowerStatus"
  "ea28f7c3c138ef8732745d1ce16b8aaf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathFollowerStatus>)))
  "Returns full string definition for message of type '<PathFollowerStatus>"
  (cl:format cl:nil "Header header~%~%#how many steps we need to reach the goal~%int64 num_steps_to_goal ~%~%#2D robot pose (x,y, theta)~%float32[3] robot_pose_2d~%~%#2D target of the robot in the local frame~%float32[3] local_target_2d~%~%#command output by the follower~%geometry_msgs/Twist cmd_vel~%~%# status can be either~%# \"goal_reached\":    when the path is empty~%# \"initial_turning\": when the robot initially rotates to align with the path~%# \"cruising\":        when the robot is tracking the path with the regular controller~%# \"finalizing\":      when the robot reached the final position, and adjusts the orientation~%# \"error\":           when the robot cannot reach the goal because no local target could be computed ~%string status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathFollowerStatus)))
  "Returns full string definition for message of type 'PathFollowerStatus"
  (cl:format cl:nil "Header header~%~%#how many steps we need to reach the goal~%int64 num_steps_to_goal ~%~%#2D robot pose (x,y, theta)~%float32[3] robot_pose_2d~%~%#2D target of the robot in the local frame~%float32[3] local_target_2d~%~%#command output by the follower~%geometry_msgs/Twist cmd_vel~%~%# status can be either~%# \"goal_reached\":    when the path is empty~%# \"initial_turning\": when the robot initially rotates to align with the path~%# \"cruising\":        when the robot is tracking the path with the regular controller~%# \"finalizing\":      when the robot reached the final position, and adjusts the orientation~%# \"error\":           when the robot cannot reach the goal because no local target could be computed ~%string status~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathFollowerStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_pose_2d) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'local_target_2d) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd_vel))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathFollowerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'PathFollowerStatus
    (cl:cons ':header (header msg))
    (cl:cons ':num_steps_to_goal (num_steps_to_goal msg))
    (cl:cons ':robot_pose_2d (robot_pose_2d msg))
    (cl:cons ':local_target_2d (local_target_2d msg))
    (cl:cons ':cmd_vel (cmd_vel msg))
    (cl:cons ':status (status msg))
))
