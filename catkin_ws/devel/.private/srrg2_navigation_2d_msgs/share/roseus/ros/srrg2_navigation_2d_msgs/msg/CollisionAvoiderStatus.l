;; Auto-generated. Do not edit!


(when (boundp 'srrg2_navigation_2d_msgs::CollisionAvoiderStatus)
  (if (not (find-package "SRRG2_NAVIGATION_2D_MSGS"))
    (make-package "SRRG2_NAVIGATION_2D_MSGS"))
  (shadow 'CollisionAvoiderStatus (find-package "SRRG2_NAVIGATION_2D_MSGS")))
(unless (find-package "SRRG2_NAVIGATION_2D_MSGS::COLLISIONAVOIDERSTATUS")
  (make-package "SRRG2_NAVIGATION_2D_MSGS::COLLISIONAVOIDERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude CollisionAvoiderStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass srrg2_navigation_2d_msgs::CollisionAvoiderStatus
  :super ros::object
  :slots (_header _cmd_vel_input _cmd_vel_output _status ))

(defmethod srrg2_navigation_2d_msgs::CollisionAvoiderStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cmd_vel_input __cmd_vel_input) (instance geometry_msgs::Twist :init))
    ((:cmd_vel_output __cmd_vel_output) (instance geometry_msgs::Twist :init))
    ((:status __status) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _cmd_vel_input __cmd_vel_input)
   (setq _cmd_vel_output __cmd_vel_output)
   (setq _status (string __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cmd_vel_input
   (&rest __cmd_vel_input)
   (if (keywordp (car __cmd_vel_input))
       (send* _cmd_vel_input __cmd_vel_input)
     (progn
       (if __cmd_vel_input (setq _cmd_vel_input (car __cmd_vel_input)))
       _cmd_vel_input)))
  (:cmd_vel_output
   (&rest __cmd_vel_output)
   (if (keywordp (car __cmd_vel_output))
       (send* _cmd_vel_output __cmd_vel_output)
     (progn
       (if __cmd_vel_output (setq _cmd_vel_output (car __cmd_vel_output)))
       _cmd_vel_output)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Twist _cmd_vel_input
    (send _cmd_vel_input :serialization-length)
    ;; geometry_msgs/Twist _cmd_vel_output
    (send _cmd_vel_output :serialization-length)
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Twist _cmd_vel_input
       (send _cmd_vel_input :serialize s)
     ;; geometry_msgs/Twist _cmd_vel_output
       (send _cmd_vel_output :serialize s)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Twist _cmd_vel_input
     (send _cmd_vel_input :deserialize buf ptr-) (incf ptr- (send _cmd_vel_input :serialization-length))
   ;; geometry_msgs/Twist _cmd_vel_output
     (send _cmd_vel_output :deserialize buf ptr-) (incf ptr- (send _cmd_vel_output :serialization-length))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get srrg2_navigation_2d_msgs::CollisionAvoiderStatus :md5sum-) "413ea057b6949dd5b85094f0459afba5")
(setf (get srrg2_navigation_2d_msgs::CollisionAvoiderStatus :datatype-) "srrg2_navigation_2d_msgs/CollisionAvoiderStatus")
(setf (get srrg2_navigation_2d_msgs::CollisionAvoiderStatus :definition-)
      "Header header

#command input to the collision avoider
geometry_msgs/Twist cmd_vel_input

#command output by the collision avoider
geometry_msgs/Twist cmd_vel_output


# status can be either
# \"clear\":    when the obstacles are fare enough and the avoider does not kick in
# \"adjusting\": when the obstacles affect the control, but don't block the robot
# \"blocked\":   when the robot stopped due to a dynamic obstacle

string status
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

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :srrg2_navigation_2d_msgs/CollisionAvoiderStatus "413ea057b6949dd5b85094f0459afba5")


