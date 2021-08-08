# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from srrg2_navigation_2d_msgs/LocalPathPlannerStatus.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class LocalPathPlannerStatus(genpy.Message):
  _md5sum = "ea28f7c3c138ef8732745d1ce16b8aaf"
  _type = "srrg2_navigation_2d_msgs/LocalPathPlannerStatus"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header

#how many steps we need to reach the goal
int64 num_steps_to_goal 

#2D robot pose (x,y, theta)
float32[3] robot_pose_2d

#2D target of the robot in the local frame
float32[3] local_target_2d

#command output by the follower
geometry_msgs/Twist cmd_vel

# status can be either
# "goal_reached":    when the path is empty
# "initial_turning": when the robot initially rotates to align with the path
# "cruising":        when the robot is tracking the path with the regular controller
# "finalizing":      when the robot reached the final position, and adjusts the orientation
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
float64 z"""
  __slots__ = ['header','num_steps_to_goal','robot_pose_2d','local_target_2d','cmd_vel','status']
  _slot_types = ['std_msgs/Header','int64','float32[3]','float32[3]','geometry_msgs/Twist','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,num_steps_to_goal,robot_pose_2d,local_target_2d,cmd_vel,status

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LocalPathPlannerStatus, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.num_steps_to_goal is None:
        self.num_steps_to_goal = 0
      if self.robot_pose_2d is None:
        self.robot_pose_2d = [0.] * 3
      if self.local_target_2d is None:
        self.local_target_2d = [0.] * 3
      if self.cmd_vel is None:
        self.cmd_vel = geometry_msgs.msg.Twist()
      if self.status is None:
        self.status = ''
    else:
      self.header = std_msgs.msg.Header()
      self.num_steps_to_goal = 0
      self.robot_pose_2d = [0.] * 3
      self.local_target_2d = [0.] * 3
      self.cmd_vel = geometry_msgs.msg.Twist()
      self.status = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.num_steps_to_goal
      buff.write(_get_struct_q().pack(_x))
      buff.write(_get_struct_3f().pack(*self.robot_pose_2d))
      buff.write(_get_struct_3f().pack(*self.local_target_2d))
      _x = self
      buff.write(_get_struct_6d().pack(_x.cmd_vel.linear.x, _x.cmd_vel.linear.y, _x.cmd_vel.linear.z, _x.cmd_vel.angular.x, _x.cmd_vel.angular.y, _x.cmd_vel.angular.z))
      _x = self.status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.cmd_vel is None:
        self.cmd_vel = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 8
      (self.num_steps_to_goal,) = _get_struct_q().unpack(str[start:end])
      start = end
      end += 12
      self.robot_pose_2d = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.local_target_2d = _get_struct_3f().unpack(str[start:end])
      _x = self
      start = end
      end += 48
      (_x.cmd_vel.linear.x, _x.cmd_vel.linear.y, _x.cmd_vel.linear.z, _x.cmd_vel.angular.x, _x.cmd_vel.angular.y, _x.cmd_vel.angular.z,) = _get_struct_6d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.num_steps_to_goal
      buff.write(_get_struct_q().pack(_x))
      buff.write(self.robot_pose_2d.tostring())
      buff.write(self.local_target_2d.tostring())
      _x = self
      buff.write(_get_struct_6d().pack(_x.cmd_vel.linear.x, _x.cmd_vel.linear.y, _x.cmd_vel.linear.z, _x.cmd_vel.angular.x, _x.cmd_vel.angular.y, _x.cmd_vel.angular.z))
      _x = self.status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.cmd_vel is None:
        self.cmd_vel = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 8
      (self.num_steps_to_goal,) = _get_struct_q().unpack(str[start:end])
      start = end
      end += 12
      self.robot_pose_2d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.local_target_2d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      _x = self
      start = end
      end += 48
      (_x.cmd_vel.linear.x, _x.cmd_vel.linear.y, _x.cmd_vel.linear.z, _x.cmd_vel.angular.x, _x.cmd_vel.angular.y, _x.cmd_vel.angular.z,) = _get_struct_6d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d
_struct_q = None
def _get_struct_q():
    global _struct_q
    if _struct_q is None:
        _struct_q = struct.Struct("<q")
    return _struct_q
