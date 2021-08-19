// Generated by gencpp from file pick_e_delivery/Pose.msg
// DO NOT EDIT!


#ifndef PICK_E_DELIVERY_MESSAGE_POSE_H
#define PICK_E_DELIVERY_MESSAGE_POSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pick_e_delivery
{
template <class ContainerAllocator>
struct Pose_
{
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
    : x(0.0)
    , y(0.0)
    , yaw(0.0)
    , status(0)
    , status_msg()  {
    }
  Pose_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , yaw(0.0)
    , status(0)
    , status_msg(_alloc)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef int32_t _status_type;
  _status_type status;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_msg_type;
  _status_msg_type status_msg;





  typedef boost::shared_ptr< ::pick_e_delivery::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pick_e_delivery::Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef ::pick_e_delivery::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::pick_e_delivery::Pose > PosePtr;
typedef boost::shared_ptr< ::pick_e_delivery::Pose const> PoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pick_e_delivery::Pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pick_e_delivery::Pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pick_e_delivery::Pose_<ContainerAllocator1> & lhs, const ::pick_e_delivery::Pose_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.yaw == rhs.yaw &&
    lhs.status == rhs.status &&
    lhs.status_msg == rhs.status_msg;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pick_e_delivery::Pose_<ContainerAllocator1> & lhs, const ::pick_e_delivery::Pose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pick_e_delivery

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pick_e_delivery::Pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pick_e_delivery::Pose_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pick_e_delivery::Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pick_e_delivery::Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pick_e_delivery::Pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pick_e_delivery::Pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pick_e_delivery::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fa1b81ed9024769c496d819f689530fc";
  }

  static const char* value(const ::pick_e_delivery::Pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfa1b81ed9024769cULL;
  static const uint64_t static_value2 = 0x496d819f689530fcULL;
};

template<class ContainerAllocator>
struct DataType< ::pick_e_delivery::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pick_e_delivery/Pose";
  }

  static const char* value(const ::pick_e_delivery::Pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pick_e_delivery::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 yaw\n"
"int32 status\n"
"string status_msg\n"
;
  }

  static const char* value(const ::pick_e_delivery::Pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pick_e_delivery::Pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.yaw);
      stream.next(m.status);
      stream.next(m.status_msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pick_e_delivery::Pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pick_e_delivery::Pose_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "status: ";
    Printer<int32_t>::stream(s, indent + "  ", v.status);
    s << indent << "status_msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status_msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PICK_E_DELIVERY_MESSAGE_POSE_H
