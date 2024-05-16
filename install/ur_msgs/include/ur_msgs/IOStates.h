// Generated by gencpp from file ur_msgs/IOStates.msg
// DO NOT EDIT!


#ifndef UR_MSGS_MESSAGE_IOSTATES_H
#define UR_MSGS_MESSAGE_IOSTATES_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ur_msgs/Digital.h>
#include <ur_msgs/Digital.h>
#include <ur_msgs/Digital.h>
#include <ur_msgs/Analog.h>
#include <ur_msgs/Analog.h>

namespace ur_msgs
{
template <class ContainerAllocator>
struct IOStates_
{
  typedef IOStates_<ContainerAllocator> Type;

  IOStates_()
    : digital_in_states()
    , digital_out_states()
    , flag_states()
    , analog_in_states()
    , analog_out_states()  {
    }
  IOStates_(const ContainerAllocator& _alloc)
    : digital_in_states(_alloc)
    , digital_out_states(_alloc)
    , flag_states(_alloc)
    , analog_in_states(_alloc)
    , analog_out_states(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::ur_msgs::Digital_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ur_msgs::Digital_<ContainerAllocator> >> _digital_in_states_type;
  _digital_in_states_type digital_in_states;

   typedef std::vector< ::ur_msgs::Digital_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ur_msgs::Digital_<ContainerAllocator> >> _digital_out_states_type;
  _digital_out_states_type digital_out_states;

   typedef std::vector< ::ur_msgs::Digital_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ur_msgs::Digital_<ContainerAllocator> >> _flag_states_type;
  _flag_states_type flag_states;

   typedef std::vector< ::ur_msgs::Analog_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ur_msgs::Analog_<ContainerAllocator> >> _analog_in_states_type;
  _analog_in_states_type analog_in_states;

   typedef std::vector< ::ur_msgs::Analog_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ur_msgs::Analog_<ContainerAllocator> >> _analog_out_states_type;
  _analog_out_states_type analog_out_states;





  typedef boost::shared_ptr< ::ur_msgs::IOStates_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur_msgs::IOStates_<ContainerAllocator> const> ConstPtr;

}; // struct IOStates_

typedef ::ur_msgs::IOStates_<std::allocator<void> > IOStates;

typedef boost::shared_ptr< ::ur_msgs::IOStates > IOStatesPtr;
typedef boost::shared_ptr< ::ur_msgs::IOStates const> IOStatesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur_msgs::IOStates_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur_msgs::IOStates_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ur_msgs::IOStates_<ContainerAllocator1> & lhs, const ::ur_msgs::IOStates_<ContainerAllocator2> & rhs)
{
  return lhs.digital_in_states == rhs.digital_in_states &&
    lhs.digital_out_states == rhs.digital_out_states &&
    lhs.flag_states == rhs.flag_states &&
    lhs.analog_in_states == rhs.analog_in_states &&
    lhs.analog_out_states == rhs.analog_out_states;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ur_msgs::IOStates_<ContainerAllocator1> & lhs, const ::ur_msgs::IOStates_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ur_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::IOStates_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur_msgs::IOStates_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::IOStates_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur_msgs::IOStates_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::IOStates_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur_msgs::IOStates_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur_msgs::IOStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b341cc0bc3ea976e9cacf81c26adeb88";
  }

  static const char* value(const ::ur_msgs::IOStates_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb341cc0bc3ea976eULL;
  static const uint64_t static_value2 = 0x9cacf81c26adeb88ULL;
};

template<class ContainerAllocator>
struct DataType< ::ur_msgs::IOStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur_msgs/IOStates";
  }

  static const char* value(const ::ur_msgs::IOStates_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur_msgs::IOStates_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Digital[] digital_in_states\n"
"Digital[] digital_out_states\n"
"Digital[] flag_states\n"
"Analog[] analog_in_states\n"
"Analog[] analog_out_states\n"
"\n"
"================================================================================\n"
"MSG: ur_msgs/Digital\n"
"uint8 pin\n"
"bool state\n"
"\n"
"================================================================================\n"
"MSG: ur_msgs/Analog\n"
"uint8 CURRENT=0\n"
"uint8 VOLTAGE=1\n"
"\n"
"uint8 pin\n"
"uint8 domain # can be VOLTAGE or CURRENT\n"
"float32 state\n"
;
  }

  static const char* value(const ::ur_msgs::IOStates_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur_msgs::IOStates_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.digital_in_states);
      stream.next(m.digital_out_states);
      stream.next(m.flag_states);
      stream.next(m.analog_in_states);
      stream.next(m.analog_out_states);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IOStates_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur_msgs::IOStates_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur_msgs::IOStates_<ContainerAllocator>& v)
  {
    s << indent << "digital_in_states[]" << std::endl;
    for (size_t i = 0; i < v.digital_in_states.size(); ++i)
    {
      s << indent << "  digital_in_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ur_msgs::Digital_<ContainerAllocator> >::stream(s, indent + "    ", v.digital_in_states[i]);
    }
    s << indent << "digital_out_states[]" << std::endl;
    for (size_t i = 0; i < v.digital_out_states.size(); ++i)
    {
      s << indent << "  digital_out_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ur_msgs::Digital_<ContainerAllocator> >::stream(s, indent + "    ", v.digital_out_states[i]);
    }
    s << indent << "flag_states[]" << std::endl;
    for (size_t i = 0; i < v.flag_states.size(); ++i)
    {
      s << indent << "  flag_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ur_msgs::Digital_<ContainerAllocator> >::stream(s, indent + "    ", v.flag_states[i]);
    }
    s << indent << "analog_in_states[]" << std::endl;
    for (size_t i = 0; i < v.analog_in_states.size(); ++i)
    {
      s << indent << "  analog_in_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ur_msgs::Analog_<ContainerAllocator> >::stream(s, indent + "    ", v.analog_in_states[i]);
    }
    s << indent << "analog_out_states[]" << std::endl;
    for (size_t i = 0; i < v.analog_out_states.size(); ++i)
    {
      s << indent << "  analog_out_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ur_msgs::Analog_<ContainerAllocator> >::stream(s, indent + "    ", v.analog_out_states[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR_MSGS_MESSAGE_IOSTATES_H
