// Generated by gencpp from file pick_e_delivery/setWaitPackIntervalResponse.msg
// DO NOT EDIT!


#ifndef PICK_E_DELIVERY_MESSAGE_SETWAITPACKINTERVALRESPONSE_H
#define PICK_E_DELIVERY_MESSAGE_SETWAITPACKINTERVALRESPONSE_H


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
struct setWaitPackIntervalResponse_
{
  typedef setWaitPackIntervalResponse_<ContainerAllocator> Type;

  setWaitPackIntervalResponse_()
    : period(0.0)  {
    }
  setWaitPackIntervalResponse_(const ContainerAllocator& _alloc)
    : period(0.0)  {
  (void)_alloc;
    }



   typedef float _period_type;
  _period_type period;





  typedef boost::shared_ptr< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> const> ConstPtr;

}; // struct setWaitPackIntervalResponse_

typedef ::pick_e_delivery::setWaitPackIntervalResponse_<std::allocator<void> > setWaitPackIntervalResponse;

typedef boost::shared_ptr< ::pick_e_delivery::setWaitPackIntervalResponse > setWaitPackIntervalResponsePtr;
typedef boost::shared_ptr< ::pick_e_delivery::setWaitPackIntervalResponse const> setWaitPackIntervalResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator1> & lhs, const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator2> & rhs)
{
  return lhs.period == rhs.period;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator1> & lhs, const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pick_e_delivery

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3bfcf350fbd9f1aad223e564a8c1f0dd";
  }

  static const char* value(const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3bfcf350fbd9f1aaULL;
  static const uint64_t static_value2 = 0xd223e564a8c1f0ddULL;
};

template<class ContainerAllocator>
struct DataType< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pick_e_delivery/setWaitPackIntervalResponse";
  }

  static const char* value(const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 period\n"
;
  }

  static const char* value(const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.period);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct setWaitPackIntervalResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pick_e_delivery::setWaitPackIntervalResponse_<ContainerAllocator>& v)
  {
    s << indent << "period: ";
    Printer<float>::stream(s, indent + "  ", v.period);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PICK_E_DELIVERY_MESSAGE_SETWAITPACKINTERVALRESPONSE_H