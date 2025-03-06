// Generated by gencpp from file rtabmap_msgs/PublishMapResponse.msg
// DO NOT EDIT!


#ifndef RTABMAP_MSGS_MESSAGE_PUBLISHMAPRESPONSE_H
#define RTABMAP_MSGS_MESSAGE_PUBLISHMAPRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rtabmap_msgs
{
template <class ContainerAllocator>
struct PublishMapResponse_
{
  typedef PublishMapResponse_<ContainerAllocator> Type;

  PublishMapResponse_()
    {
    }
  PublishMapResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> const> ConstPtr;

}; // struct PublishMapResponse_

typedef ::rtabmap_msgs::PublishMapResponse_<std::allocator<void> > PublishMapResponse;

typedef boost::shared_ptr< ::rtabmap_msgs::PublishMapResponse > PublishMapResponsePtr;
typedef boost::shared_ptr< ::rtabmap_msgs::PublishMapResponse const> PublishMapResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rtabmap_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'rtabmap_msgs': ['/home/robond/ROS_Udacity/src/rtabmap_ros/rtabmap_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rtabmap_msgs/PublishMapResponse";
  }

  static const char* value(const ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
";
  }

  static const char* value(const ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PublishMapResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::rtabmap_msgs::PublishMapResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // RTABMAP_MSGS_MESSAGE_PUBLISHMAPRESPONSE_H
