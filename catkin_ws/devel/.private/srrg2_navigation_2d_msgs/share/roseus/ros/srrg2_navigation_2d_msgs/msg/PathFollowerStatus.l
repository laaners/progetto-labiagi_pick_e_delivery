;; Auto-generated. Do not edit!


(when (boundp 'srrg2_navigation_2d_msgs::PathFollowerStatus)
  (if (not (find-package "SRRG2_NAVIGATION_2D_MSGS"))
    (make-package "SRRG2_NAVIGATION_2D_MSGS"))
  (shadow 'PathFollowerStatus (find-package "SRRG2_NAVIGATION_2D_MSGS")))
(unless (find-package "SRRG2_NAVIGATION_2D_MSGS::PATHFOLLOWERSTATUS")
  (make-package "SRRG2_NAVIGATION_2D_MSGS::PATHFOLLOWERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude PathFollowerStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass srrg2_navigation_2d_msgs::PathFollowerStatus
  :super ros::object
  :slots (_header _num_steps_to_goal _robot_pose_2d _local_target_2d _cmd_vel _status ))

(defmethod srrg2_navigation_2d_msgs::PathFollowerStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_steps_to_goal __num_steps_to_goal) 0)
    ((:robot_pose_2d __robot_pose_2d) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:local_target_2d __local_target_2d) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:cmd_vel __cmd_vel) (instance geometry_msgs::Twist :init))
    ((:status __status) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_steps_to_goal (round __num_steps_to_goal))
   (setq _robot_pose_2d __robot_pose_2d)
   (setq _local_target_2d __local_target_2d)
   (setq _cmd_vel __cmd_vel)
   (setq _status (string __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_steps_to_goal
   (&optional __num_steps_to_goal)
   (if __num_steps_to_goal (setq _num_steps_to_goal __num_steps_to_goal)) _num_steps_to_goal)
  (:robot_pose_2d
   (&optional __robot_pose_2d)
   (if __robot_pose_2d (setq _robot_pose_2d __robot_pose_2d)) _robot_pose_2d)
  (:local_target_2d
   (&optional __local_target_2d)
   (if __local_target_2d (setq _local_target_2d __local_target_2d)) _local_target_2d)
  (:cmd_vel
   (&rest __cmd_vel)
   (if (keywordp (car __cmd_vel))
       (send* _cmd_vel __cmd_vel)
     (progn
       (if __cmd_vel (setq _cmd_vel (car __cmd_vel)))
       _cmd_vel)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _num_steps_to_goal
    8
    ;; float32[3] _robot_pose_2d
    (* 4    3)
    ;; float32[3] _local_target_2d
    (* 4    3)
    ;; geometry_msgs/Twist _cmd_vel
    (send _cmd_vel :serialization-length)
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _num_steps_to_goal
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_steps_to_goal (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_steps_to_goal) (= (length (_num_steps_to_goal . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_steps_to_goal . bv) 0) 0) s)
              (write-long (ash (elt (_num_steps_to_goal . bv) 1) -1) s))
             ((and (class _num_steps_to_goal) (= (length (_num_steps_to_goal . bv)) 1)) ;; big1
              (write-long (elt (_num_steps_to_goal . bv) 0) s)
              (write-long (if (>= _num_steps_to_goal 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_steps_to_goal s)(write-long (if (>= _num_steps_to_goal 0) 0 #xffffffff) s)))
     ;; float32[3] _robot_pose_2d
     (dotimes (i 3)
       (sys::poke (elt _robot_pose_2d i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _local_target_2d
     (dotimes (i 3)
       (sys::poke (elt _local_target_2d i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; geometry_msgs/Twist _cmd_vel
       (send _cmd_vel :serialize s)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _num_steps_to_goal
#+(or :alpha :irix6 :x86_64)
      (setf _num_steps_to_goal (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_steps_to_goal (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32[3] _robot_pose_2d
   (dotimes (i (length _robot_pose_2d))
     (setf (elt _robot_pose_2d i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _local_target_2d
   (dotimes (i (length _local_target_2d))
     (setf (elt _local_target_2d i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; geometry_msgs/Twist _cmd_vel
     (send _cmd_vel :deserialize buf ptr-) (incf ptr- (send _cmd_vel :serialization-length))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get srrg2_navigation_2d_msgs::PathFollowerStatus :md5sum-) "ea28f7c3c138ef8732745d1ce16b8aaf")
(setf (get srrg2_navigation_2d_msgs::PathFollowerStatus :datatype-) "srrg2_navigation_2d_msgs/PathFollowerStatus")
(setf (get srrg2_navigation_2d_msgs::PathFollowerStatus :definition-)
      "Header header

#how many steps we need to reach the goal
int64 num_steps_to_goal 

#2D robot pose (x,y, theta)
float32[3] robot_pose_2d

#2D target of the robot in the local frame
float32[3] local_target_2d

#command output by the follower
geometry_msgs/Twist cmd_vel

# status can be either
# \"goal_reached\":    when the path is empty
# \"initial_turning\": when the robot initially rotates to align with the path
# \"cruising\":        when the robot is tracking the path with the regular controller
# \"finalizing\":      when the robot reached the final position, and adjusts the orientation
# \"error\":           when the robot cannot reach the goal because no local target could be computed 
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



(provide :srrg2_navigation_2d_msgs/PathFollowerStatus "ea28f7c3c138ef8732745d1ce16b8aaf")


