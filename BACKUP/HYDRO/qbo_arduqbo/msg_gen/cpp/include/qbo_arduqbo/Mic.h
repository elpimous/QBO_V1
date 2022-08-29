/* Auto-generated by genmsg_cpp for file /opt/ros/hydro/stacks/qbo_arduqbo/msg/Mic.msg */
#ifndef QBO_ARDUQBO_MESSAGE_MIC_H
#define QBO_ARDUQBO_MESSAGE_MIC_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace qbo_arduqbo
{
template <class ContainerAllocator>
struct Mic_ {
  typedef Mic_<ContainerAllocator> Type;

  Mic_()
  : header()
  , mic(0)
  {
  }

  Mic_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , mic(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _mic_type;
  uint8_t mic;


  typedef boost::shared_ptr< ::qbo_arduqbo::Mic_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qbo_arduqbo::Mic_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Mic
typedef  ::qbo_arduqbo::Mic_<std::allocator<void> > Mic;

typedef boost::shared_ptr< ::qbo_arduqbo::Mic> MicPtr;
typedef boost::shared_ptr< ::qbo_arduqbo::Mic const> MicConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::qbo_arduqbo::Mic_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::qbo_arduqbo::Mic_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace qbo_arduqbo

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::qbo_arduqbo::Mic_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::qbo_arduqbo::Mic_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::qbo_arduqbo::Mic_<ContainerAllocator> > {
  static const char* value() 
  {
    return "53acd43451f96e661169eddc90448bdc";
  }

  static const char* value(const  ::qbo_arduqbo::Mic_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x53acd43451f96e66ULL;
  static const uint64_t static_value2 = 0x1169eddc90448bdcULL;
};

template<class ContainerAllocator>
struct DataType< ::qbo_arduqbo::Mic_<ContainerAllocator> > {
  static const char* value() 
  {
    return "qbo_arduqbo/Mic";
  }

  static const char* value(const  ::qbo_arduqbo::Mic_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::qbo_arduqbo::Mic_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# Software License Agreement (LGPL v2.1 License)\n\
#\n\
# Copyright (c) 2012 Thecorpora, Inc.\n\
#\n\
# This library is free software; you can redistribute it and/or modify \n\
# it under the terms of the GNU Lesser General Public License as published\n\
# by the Free Software Foundation; either version 2.1 of the License, \n\
# or (at your option) any later version.\n\
#\n\
# This library is distributed in the hope that it will be useful, but \n\
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY \n\
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public \n\
# License for more details.\n\
#  \n\
# You should have received a copy of the GNU General Public License \n\
# along with this program; if not, write to the Free Software \n\
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, \n\
# MA 02110-1301, USA.# Software License Agreement (LGPL v2.1 License)\n\
#\n\
# Copyright (c) 2012 Thecorpora, Inc.\n\
#\n\
# This library is free software; you can redistribute it and/or modify \n\
# it under the terms of the GNU Lesser General Public License as published\n\
# by the Free Software Foundation; either version 2.1 of the License, \n\
# or (at your option) any later version.\n\
#\n\
# This library is distributed in the hope that it will be useful, but \n\
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY \n\
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public \n\
# License for more details.\n\
#  \n\
# You should have received a copy of the GNU General Public License \n\
# along with this program; if not, write to the Free Software \n\
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, \n\
# MA 02110-1301, USA.\n\
\n\
Header header\n\
uint8 mic\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::qbo_arduqbo::Mic_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::qbo_arduqbo::Mic_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::qbo_arduqbo::Mic_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::qbo_arduqbo::Mic_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.mic);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Mic_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qbo_arduqbo::Mic_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::qbo_arduqbo::Mic_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "mic: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mic);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QBO_ARDUQBO_MESSAGE_MIC_H

