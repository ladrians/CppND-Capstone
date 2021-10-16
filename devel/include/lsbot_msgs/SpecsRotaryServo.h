// Generated by gencpp from file lsbot_msgs/SpecsRotaryServo.msg
// DO NOT EDIT!


#ifndef LSBOT_MSGS_MESSAGE_SPECSROTARYSERVO_H
#define LSBOT_MSGS_MESSAGE_SPECSROTARYSERVO_H

#include <ros/service_traits.h>


#include <lsbot_msgs/SpecsRotaryServoRequest.h>
#include <lsbot_msgs/SpecsRotaryServoResponse.h>


namespace lsbot_msgs
{

struct SpecsRotaryServo
{

typedef SpecsRotaryServoRequest Request;
typedef SpecsRotaryServoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SpecsRotaryServo
} // namespace lsbot_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::lsbot_msgs::SpecsRotaryServo > {
  static const char* value()
  {
    return "c99b39095d63ff4cbaed9c5a6eec7d20";
  }

  static const char* value(const ::lsbot_msgs::SpecsRotaryServo&) { return value(); }
};

template<>
struct DataType< ::lsbot_msgs::SpecsRotaryServo > {
  static const char* value()
  {
    return "lsbot_msgs/SpecsRotaryServo";
  }

  static const char* value(const ::lsbot_msgs::SpecsRotaryServo&) { return value(); }
};


// service_traits::MD5Sum< ::lsbot_msgs::SpecsRotaryServoRequest> should match
// service_traits::MD5Sum< ::lsbot_msgs::SpecsRotaryServo >
template<>
struct MD5Sum< ::lsbot_msgs::SpecsRotaryServoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::lsbot_msgs::SpecsRotaryServo >::value();
  }
  static const char* value(const ::lsbot_msgs::SpecsRotaryServoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::lsbot_msgs::SpecsRotaryServoRequest> should match
// service_traits::DataType< ::lsbot_msgs::SpecsRotaryServo >
template<>
struct DataType< ::lsbot_msgs::SpecsRotaryServoRequest>
{
  static const char* value()
  {
    return DataType< ::lsbot_msgs::SpecsRotaryServo >::value();
  }
  static const char* value(const ::lsbot_msgs::SpecsRotaryServoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::lsbot_msgs::SpecsRotaryServoResponse> should match
// service_traits::MD5Sum< ::lsbot_msgs::SpecsRotaryServo >
template<>
struct MD5Sum< ::lsbot_msgs::SpecsRotaryServoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::lsbot_msgs::SpecsRotaryServo >::value();
  }
  static const char* value(const ::lsbot_msgs::SpecsRotaryServoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::lsbot_msgs::SpecsRotaryServoResponse> should match
// service_traits::DataType< ::lsbot_msgs::SpecsRotaryServo >
template<>
struct DataType< ::lsbot_msgs::SpecsRotaryServoResponse>
{
  static const char* value()
  {
    return DataType< ::lsbot_msgs::SpecsRotaryServo >::value();
  }
  static const char* value(const ::lsbot_msgs::SpecsRotaryServoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LSBOT_MSGS_MESSAGE_SPECSROTARYSERVO_H
