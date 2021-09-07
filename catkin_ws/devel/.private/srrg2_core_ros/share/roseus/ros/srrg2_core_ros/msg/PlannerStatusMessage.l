;; Auto-generated. Do not edit!


(when (boundp 'srrg2_core_ros::PlannerStatusMessage)
  (if (not (find-package "SRRG2_CORE_ROS"))
    (make-package "SRRG2_CORE_ROS"))
  (shadow 'PlannerStatusMessage (find-package "SRRG2_CORE_ROS")))
(unless (find-package "SRRG2_CORE_ROS::PLANNERSTATUSMESSAGE")
  (make-package "SRRG2_CORE_ROS::PLANNERSTATUSMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude PlannerStatusMessage.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass srrg2_core_ros::PlannerStatusMessage
  :super ros::object
  :slots (_header _status _a_star_status _path_type _cost_to_global_goal _distance_to_global_goal _distance_to_local_goal ))

(defmethod srrg2_core_ros::PlannerStatusMessage
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:status __status) "")
    ((:a_star_status __a_star_status) 0)
    ((:path_type __path_type) 0)
    ((:cost_to_global_goal __cost_to_global_goal) 0.0)
    ((:distance_to_global_goal __distance_to_global_goal) 0.0)
    ((:distance_to_local_goal __distance_to_local_goal) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _status (string __status))
   (setq _a_star_status (round __a_star_status))
   (setq _path_type (round __path_type))
   (setq _cost_to_global_goal (float __cost_to_global_goal))
   (setq _distance_to_global_goal (float __distance_to_global_goal))
   (setq _distance_to_local_goal (float __distance_to_local_goal))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:a_star_status
   (&optional __a_star_status)
   (if __a_star_status (setq _a_star_status __a_star_status)) _a_star_status)
  (:path_type
   (&optional __path_type)
   (if __path_type (setq _path_type __path_type)) _path_type)
  (:cost_to_global_goal
   (&optional __cost_to_global_goal)
   (if __cost_to_global_goal (setq _cost_to_global_goal __cost_to_global_goal)) _cost_to_global_goal)
  (:distance_to_global_goal
   (&optional __distance_to_global_goal)
   (if __distance_to_global_goal (setq _distance_to_global_goal __distance_to_global_goal)) _distance_to_global_goal)
  (:distance_to_local_goal
   (&optional __distance_to_local_goal)
   (if __distance_to_local_goal (setq _distance_to_local_goal __distance_to_local_goal)) _distance_to_local_goal)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _status
    4 (length _status)
    ;; int32 _a_star_status
    4
    ;; int32 _path_type
    4
    ;; float32 _cost_to_global_goal
    4
    ;; float32 _distance_to_global_goal
    4
    ;; float32 _distance_to_local_goal
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; int32 _a_star_status
       (write-long _a_star_status s)
     ;; int32 _path_type
       (write-long _path_type s)
     ;; float32 _cost_to_global_goal
       (sys::poke _cost_to_global_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance_to_global_goal
       (sys::poke _distance_to_global_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance_to_local_goal
       (sys::poke _distance_to_local_goal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _a_star_status
     (setq _a_star_status (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _path_type
     (setq _path_type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _cost_to_global_goal
     (setq _cost_to_global_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance_to_global_goal
     (setq _distance_to_global_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance_to_local_goal
     (setq _distance_to_local_goal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get srrg2_core_ros::PlannerStatusMessage :md5sum-) "cde3a14282df6b919fe6c0fcba349400")
(setf (get srrg2_core_ros::PlannerStatusMessage :datatype-) "srrg2_core_ros/PlannerStatusMessage")
(setf (get srrg2_core_ros::PlannerStatusMessage :definition-)
      "#gg mesage for the planner status, shot continuosly by the planner

Header   header
string   status           #status of the planner (\"idle\", \"moving\", \"invalid_goal\", \"unreachable\")
int32    a_star_status    #result of the enum for the local path search
int32    path_type        #type of path 0: gradient, 1: grid
float32  cost_to_global_goal     #value of the cost function at the current location
float32  distance_to_global_goal #distance to the current location [meters]
float32  distance_to_local_goal #distance to the current location [meters]

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



(provide :srrg2_core_ros/PlannerStatusMessage "cde3a14282df6b919fe6c0fcba349400")


