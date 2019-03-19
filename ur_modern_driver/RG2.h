// Generated by gencpp from file ur_modern_driver/RG2.msg
// DO NOT EDIT!


#ifndef UR_MODERN_DRIVER_MESSAGE_RG2_H
#define UR_MODERN_DRIVER_MESSAGE_RG2_H

#include <ros/service_traits.h>


#include <ur_modern_driver/RG2Request.h>
#include <ur_modern_driver/RG2Response.h>


namespace ur_modern_driver
{

struct RG2
{

typedef RG2Request Request;
typedef RG2Response Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RG2
} // namespace ur_modern_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ur_modern_driver::RG2 > {
  static const char* value()
  {
    return "1209ffbe583478e2f1bc42f6483983ea";
  }

  static const char* value(const ::ur_modern_driver::RG2&) { return value(); }
};

template<>
struct DataType< ::ur_modern_driver::RG2 > {
  static const char* value()
  {
    return "ur_modern_driver/RG2";
  }

  static const char* value(const ::ur_modern_driver::RG2&) { return value(); }
};


// service_traits::MD5Sum< ::ur_modern_driver::RG2Request> should match 
// service_traits::MD5Sum< ::ur_modern_driver::RG2 > 
template<>
struct MD5Sum< ::ur_modern_driver::RG2Request>
{
  static const char* value()
  {
    return MD5Sum< ::ur_modern_driver::RG2 >::value();
  }
  static const char* value(const ::ur_modern_driver::RG2Request&)
  {
    return value();
  }
};

// service_traits::DataType< ::ur_modern_driver::RG2Request> should match 
// service_traits::DataType< ::ur_modern_driver::RG2 > 
template<>
struct DataType< ::ur_modern_driver::RG2Request>
{
  static const char* value()
  {
    return DataType< ::ur_modern_driver::RG2 >::value();
  }
  static const char* value(const ::ur_modern_driver::RG2Request&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ur_modern_driver::RG2Response> should match 
// service_traits::MD5Sum< ::ur_modern_driver::RG2 > 
template<>
struct MD5Sum< ::ur_modern_driver::RG2Response>
{
  static const char* value()
  {
    return MD5Sum< ::ur_modern_driver::RG2 >::value();
  }
  static const char* value(const ::ur_modern_driver::RG2Response&)
  {
    return value();
  }
};

// service_traits::DataType< ::ur_modern_driver::RG2Response> should match 
// service_traits::DataType< ::ur_modern_driver::RG2 > 
template<>
struct DataType< ::ur_modern_driver::RG2Response>
{
  static const char* value()
  {
    return DataType< ::ur_modern_driver::RG2 >::value();
  }
  static const char* value(const ::ur_modern_driver::RG2Response&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // UR_MODERN_DRIVER_MESSAGE_RG2_H
