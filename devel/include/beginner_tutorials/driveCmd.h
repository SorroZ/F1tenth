// Generated by gencpp from file beginner_tutorials/driveCmd.msg
// DO NOT EDIT!


#ifndef BEGINNER_TUTORIALS_MESSAGE_DRIVECMD_H
#define BEGINNER_TUTORIALS_MESSAGE_DRIVECMD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beginner_tutorials
{
template <class ContainerAllocator>
struct driveCmd_
{
  typedef driveCmd_<ContainerAllocator> Type;

  driveCmd_()
    : steering(0.0)
    , throttle(0.0)  {
    }
  driveCmd_(const ContainerAllocator& _alloc)
    : steering(0.0)
    , throttle(0.0)  {
  (void)_alloc;
    }



   typedef float _steering_type;
  _steering_type steering;

   typedef float _throttle_type;
  _throttle_type throttle;




  typedef boost::shared_ptr< ::beginner_tutorials::driveCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beginner_tutorials::driveCmd_<ContainerAllocator> const> ConstPtr;

}; // struct driveCmd_

typedef ::beginner_tutorials::driveCmd_<std::allocator<void> > driveCmd;

typedef boost::shared_ptr< ::beginner_tutorials::driveCmd > driveCmdPtr;
typedef boost::shared_ptr< ::beginner_tutorials::driveCmd const> driveCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::beginner_tutorials::driveCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::beginner_tutorials::driveCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace beginner_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'beginner_tutorials': ['/home/ubuntu/good_ws/src/beginner_tutorials/msg', '/home/ubuntu/good_ws/src/beginner_tutorials/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::driveCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::driveCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::driveCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "07077f1ca3b57b112f69aabcdabf600e";
  }

  static const char* value(const ::beginner_tutorials::driveCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x07077f1ca3b57b11ULL;
  static const uint64_t static_value2 = 0x2f69aabcdabf600eULL;
};

template<class ContainerAllocator>
struct DataType< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "beginner_tutorials/driveCmd";
  }

  static const char* value(const ::beginner_tutorials::driveCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 steering\n\
float32 throttle\n\
";
  }

  static const char* value(const ::beginner_tutorials::driveCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.steering);
      stream.next(m.throttle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct driveCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beginner_tutorials::driveCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beginner_tutorials::driveCmd_<ContainerAllocator>& v)
  {
    s << indent << "steering: ";
    Printer<float>::stream(s, indent + "  ", v.steering);
    s << indent << "throttle: ";
    Printer<float>::stream(s, indent + "  ", v.throttle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_DRIVECMD_H
