// Generated by gencpp from file rtabmap_msgs/SetLabelRequest.msg
// DO NOT EDIT!


#ifndef RTABMAP_MSGS_MESSAGE_SETLABELREQUEST_H
#define RTABMAP_MSGS_MESSAGE_SETLABELREQUEST_H


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
struct SetLabelRequest_
{
  typedef SetLabelRequest_<ContainerAllocator> Type;

  SetLabelRequest_()
    : node_id(0)
    , node_label()  {
    }
  SetLabelRequest_(const ContainerAllocator& _alloc)
    : node_id(0)
    , node_label(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _node_id_type;
  _node_id_type node_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _node_label_type;
  _node_label_type node_label;





  typedef boost::shared_ptr< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetLabelRequest_

typedef ::rtabmap_msgs::SetLabelRequest_<std::allocator<void> > SetLabelRequest;

typedef boost::shared_ptr< ::rtabmap_msgs::SetLabelRequest > SetLabelRequestPtr;
typedef boost::shared_ptr< ::rtabmap_msgs::SetLabelRequest const> SetLabelRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rtabmap_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rtabmap_msgs': ['/home/robond/ROS_Udacity/src/rtabmap_ros/rtabmap_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baadfb04a43ec26085eb7bebc9a80862";
  }

  static const char* value(const ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbaadfb04a43ec260ULL;
  static const uint64_t static_value2 = 0x85eb7bebc9a80862ULL;
};

template<class ContainerAllocator>
struct DataType< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rtabmap_msgs/SetLabelRequest";
  }

  static const char* value(const ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
int32 node_id\n\
string node_label\n\
";
  }

  static const char* value(const ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.node_id);
      stream.next(m.node_label);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetLabelRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rtabmap_msgs::SetLabelRequest_<ContainerAllocator>& v)
  {
    s << indent << "node_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.node_id);
    s << indent << "node_label: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.node_label);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RTABMAP_MSGS_MESSAGE_SETLABELREQUEST_H
