// Generated by gencpp from file robomaker_simulation_msgs/CancelRequest.msg
// DO NOT EDIT!


#ifndef ROBOMAKER_SIMULATION_MSGS_MESSAGE_CANCELREQUEST_H
#define ROBOMAKER_SIMULATION_MSGS_MESSAGE_CANCELREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robomaker_simulation_msgs
{
template <class ContainerAllocator>
struct CancelRequest_
{
  typedef CancelRequest_<ContainerAllocator> Type;

  CancelRequest_()
    {
    }
  CancelRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CancelRequest_

typedef ::robomaker_simulation_msgs::CancelRequest_<std::allocator<void> > CancelRequest;

typedef boost::shared_ptr< ::robomaker_simulation_msgs::CancelRequest > CancelRequestPtr;
typedef boost::shared_ptr< ::robomaker_simulation_msgs::CancelRequest const> CancelRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robomaker_simulation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'robomaker_simulation_msgs': ['/opt/workspace/AwsSilverstoneSimulationApplication/src/aws-robomaker-simulation-ros-pkgs/robomaker_simulation_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robomaker_simulation_msgs/CancelRequest";
  }

  static const char* value(const ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
\n\
";
  }

  static const char* value(const ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CancelRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::robomaker_simulation_msgs::CancelRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ROBOMAKER_SIMULATION_MSGS_MESSAGE_CANCELREQUEST_H