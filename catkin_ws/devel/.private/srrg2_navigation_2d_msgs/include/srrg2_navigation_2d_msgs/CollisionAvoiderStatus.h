// Generated by gencpp from file srrg2_navigation_2d_msgs/CollisionAvoiderStatus.msg
// DO NOT EDIT!


#ifndef SRRG2_NAVIGATION_2D_MSGS_MESSAGE_COLLISIONAVOIDERSTATUS_H
#define SRRG2_NAVIGATION_2D_MSGS_MESSAGE_COLLISIONAVOIDERSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>

namespace srrg2_navigation_2d_msgs
{
template <class ContainerAllocator>
struct CollisionAvoiderStatus_
{
  typedef CollisionAvoiderStatus_<ContainerAllocator> Type;

  CollisionAvoiderStatus_()
    : header()
    , cmd_vel_input()
    , cmd_vel_output()
    , status()  {
    }
  CollisionAvoiderStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cmd_vel_input(_alloc)
    , cmd_vel_output(_alloc)
    , status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _cmd_vel_input_type;
  _cmd_vel_input_type cmd_vel_input;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _cmd_vel_output_type;
  _cmd_vel_output_type cmd_vel_output;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> const> ConstPtr;

}; // struct CollisionAvoiderStatus_

typedef ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<std::allocator<void> > CollisionAvoiderStatus;

typedef boost::shared_ptr< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus > CollisionAvoiderStatusPtr;
typedef boost::shared_ptr< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus const> CollisionAvoiderStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator1> & lhs, const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.cmd_vel_input == rhs.cmd_vel_input &&
    lhs.cmd_vel_output == rhs.cmd_vel_output &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator1> & lhs, const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace srrg2_navigation_2d_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "413ea057b6949dd5b85094f0459afba5";
  }

  static const char* value(const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x413ea057b6949dd5ULL;
  static const uint64_t static_value2 = 0xb85094f0459afba5ULL;
};

template<class ContainerAllocator>
struct DataType< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "srrg2_navigation_2d_msgs/CollisionAvoiderStatus";
  }

  static const char* value(const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"#command input to the collision avoider\n"
"geometry_msgs/Twist cmd_vel_input\n"
"\n"
"#command output by the collision avoider\n"
"geometry_msgs/Twist cmd_vel_output\n"
"\n"
"\n"
"# status can be either\n"
"# \"clear\":    when the obstacles are fare enough and the avoider does not kick in\n"
"# \"adjusting\": when the obstacles affect the control, but don't block the robot\n"
"# \"blocked\":   when the robot stopped due to a dynamic obstacle\n"
"\n"
"string status\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cmd_vel_input);
      stream.next(m.cmd_vel_output);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CollisionAvoiderStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::srrg2_navigation_2d_msgs::CollisionAvoiderStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cmd_vel_input: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.cmd_vel_input);
    s << indent << "cmd_vel_output: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.cmd_vel_output);
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SRRG2_NAVIGATION_2D_MSGS_MESSAGE_COLLISIONAVOIDERSTATUS_H
