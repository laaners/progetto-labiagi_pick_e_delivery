// Generated by gencpp from file set_goal/NewGoal.msg
// DO NOT EDIT!


#ifndef SET_GOAL_MESSAGE_NEWGOAL_H
#define SET_GOAL_MESSAGE_NEWGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace set_goal
{
template <class ContainerAllocator>
struct NewGoal_
{
  typedef NewGoal_<ContainerAllocator> Type;

  NewGoal_()
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }
  NewGoal_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;





  typedef boost::shared_ptr< ::set_goal::NewGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::set_goal::NewGoal_<ContainerAllocator> const> ConstPtr;

}; // struct NewGoal_

typedef ::set_goal::NewGoal_<std::allocator<void> > NewGoal;

typedef boost::shared_ptr< ::set_goal::NewGoal > NewGoalPtr;
typedef boost::shared_ptr< ::set_goal::NewGoal const> NewGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::set_goal::NewGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::set_goal::NewGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::set_goal::NewGoal_<ContainerAllocator1> & lhs, const ::set_goal::NewGoal_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.theta == rhs.theta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::set_goal::NewGoal_<ContainerAllocator1> & lhs, const ::set_goal::NewGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace set_goal

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::set_goal::NewGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::set_goal::NewGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::set_goal::NewGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::set_goal::NewGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::set_goal::NewGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::set_goal::NewGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::set_goal::NewGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a130bc60ee6513855dc62ea83fcc5b20";
  }

  static const char* value(const ::set_goal::NewGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa130bc60ee651385ULL;
  static const uint64_t static_value2 = 0x5dc62ea83fcc5b20ULL;
};

template<class ContainerAllocator>
struct DataType< ::set_goal::NewGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "set_goal/NewGoal";
  }

  static const char* value(const ::set_goal::NewGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::set_goal::NewGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 theta\n"
;
  }

  static const char* value(const ::set_goal::NewGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::set_goal::NewGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NewGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::set_goal::NewGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::set_goal::NewGoal_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SET_GOAL_MESSAGE_NEWGOAL_H