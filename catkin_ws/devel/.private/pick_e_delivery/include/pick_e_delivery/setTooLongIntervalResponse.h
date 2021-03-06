// Generated by gencpp from file pick_e_delivery/setTooLongIntervalResponse.msg
// DO NOT EDIT!


#ifndef PICK_E_DELIVERY_MESSAGE_SETTOOLONGINTERVALRESPONSE_H
#define PICK_E_DELIVERY_MESSAGE_SETTOOLONGINTERVALRESPONSE_H


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
struct setTooLongIntervalResponse_
{
  typedef setTooLongIntervalResponse_<ContainerAllocator> Type;

  setTooLongIntervalResponse_()
    : period(0.0)  {
    }
  setTooLongIntervalResponse_(const ContainerAllocator& _alloc)
    : period(0.0)  {
  (void)_alloc;
    }



   typedef float _period_type;
  _period_type period;





  typedef boost::shared_ptr< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> const> ConstPtr;

}; // struct setTooLongIntervalResponse_

typedef ::pick_e_delivery::setTooLongIntervalResponse_<std::allocator<void> > setTooLongIntervalResponse;

typedef boost::shared_ptr< ::pick_e_delivery::setTooLongIntervalResponse > setTooLongIntervalResponsePtr;
typedef boost::shared_ptr< ::pick_e_delivery::setTooLongIntervalResponse const> setTooLongIntervalResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator1> & lhs, const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator2> & rhs)
{
  return lhs.period == rhs.period;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator1> & lhs, const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pick_e_delivery

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3bfcf350fbd9f1aad223e564a8c1f0dd";
  }

  static const char* value(const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3bfcf350fbd9f1aaULL;
  static const uint64_t static_value2 = 0xd223e564a8c1f0ddULL;
};

template<class ContainerAllocator>
struct DataType< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pick_e_delivery/setTooLongIntervalResponse";
  }

  static const char* value(const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 period\n"
;
  }

  static const char* value(const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.period);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct setTooLongIntervalResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pick_e_delivery::setTooLongIntervalResponse_<ContainerAllocator>& v)
  {
    s << indent << "period: ";
    Printer<float>::stream(s, indent + "  ", v.period);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PICK_E_DELIVERY_MESSAGE_SETTOOLONGINTERVALRESPONSE_H
