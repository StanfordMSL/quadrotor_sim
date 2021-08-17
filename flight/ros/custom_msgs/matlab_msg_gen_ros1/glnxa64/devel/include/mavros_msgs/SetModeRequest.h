// Generated by gencpp from file mavros_msgs/SetModeRequest.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_SETMODEREQUEST_H
#define MAVROS_MSGS_MESSAGE_SETMODEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mavros_msgs
{
template <class ContainerAllocator>
struct SetModeRequest_
{
  typedef SetModeRequest_<ContainerAllocator> Type;

  SetModeRequest_()
    : base_mode(0)
    , custom_mode()  {
    }
  SetModeRequest_(const ContainerAllocator& _alloc)
    : base_mode(0)
    , custom_mode(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _base_mode_type;
  _base_mode_type base_mode;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _custom_mode_type;
  _custom_mode_type custom_mode;



  enum {
 
    MAV_MODE_PREFLIGHT = 0u,
 
    MAV_MODE_STABILIZE_DISARMED = 80u,
 
    MAV_MODE_STABILIZE_ARMED = 208u,
 
    MAV_MODE_MANUAL_DISARMED = 64u,
 
    MAV_MODE_MANUAL_ARMED = 192u,
 
    MAV_MODE_GUIDED_DISARMED = 88u,
 
    MAV_MODE_GUIDED_ARMED = 216u,
 
    MAV_MODE_AUTO_DISARMED = 92u,
 
    MAV_MODE_AUTO_ARMED = 220u,
 
    MAV_MODE_TEST_DISARMED = 66u,
 
    MAV_MODE_TEST_ARMED = 194u,
  };


  typedef boost::shared_ptr< ::mavros_msgs::SetModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::SetModeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetModeRequest_

typedef ::mavros_msgs::SetModeRequest_<std::allocator<void> > SetModeRequest;

typedef boost::shared_ptr< ::mavros_msgs::SetModeRequest > SetModeRequestPtr;
typedef boost::shared_ptr< ::mavros_msgs::SetModeRequest const> SetModeRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::SetModeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'mavros_msgs': ['/home/lowjunen/StanfordMSL/quadrotor_sim/flight/ros/custom_msgs/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::SetModeRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::SetModeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::SetModeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d770431847cad3a8f50a81ec70ddf0e2";
  }

  static const char* value(const ::mavros_msgs::SetModeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd770431847cad3a8ULL;
  static const uint64_t static_value2 = 0xf50a81ec70ddf0e2ULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/SetModeRequest";
  }

  static const char* value(const ::mavros_msgs::SetModeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"\n"
"\n"
"\n"
"\n"
"uint8 MAV_MODE_PREFLIGHT		= 0\n"
"uint8 MAV_MODE_STABILIZE_DISARMED	= 80\n"
"uint8 MAV_MODE_STABILIZE_ARMED		= 208\n"
"uint8 MAV_MODE_MANUAL_DISARMED		= 64\n"
"uint8 MAV_MODE_MANUAL_ARMED		= 192\n"
"uint8 MAV_MODE_GUIDED_DISARMED		= 88\n"
"uint8 MAV_MODE_GUIDED_ARMED		= 216\n"
"uint8 MAV_MODE_AUTO_DISARMED		= 92\n"
"uint8 MAV_MODE_AUTO_ARMED		= 220\n"
"uint8 MAV_MODE_TEST_DISARMED		= 66\n"
"uint8 MAV_MODE_TEST_ARMED		= 194\n"
"\n"
"uint8 base_mode\n"
"string custom_mode\n"
;
  }

  static const char* value(const ::mavros_msgs::SetModeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.base_mode);
      stream.next(m.custom_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetModeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::SetModeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::SetModeRequest_<ContainerAllocator>& v)
  {
    s << indent << "base_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.base_mode);
    s << indent << "custom_mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.custom_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_SETMODEREQUEST_H
