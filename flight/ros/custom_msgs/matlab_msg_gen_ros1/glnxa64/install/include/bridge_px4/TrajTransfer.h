// Generated by gencpp from file bridge_px4/TrajTransfer.msg
// DO NOT EDIT!


#ifndef BRIDGE_PX4_MESSAGE_TRAJTRANSFER_H
#define BRIDGE_PX4_MESSAGE_TRAJTRANSFER_H

#include <ros/service_traits.h>


#include <bridge_px4/TrajTransferRequest.h>
#include <bridge_px4/TrajTransferResponse.h>


namespace bridge_px4
{

struct TrajTransfer
{

typedef TrajTransferRequest Request;
typedef TrajTransferResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TrajTransfer
} // namespace bridge_px4


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::bridge_px4::TrajTransfer > {
  static const char* value()
  {
    return "86ad9a373914f3b846b0a89d22fe4af3";
  }

  static const char* value(const ::bridge_px4::TrajTransfer&) { return value(); }
};

template<>
struct DataType< ::bridge_px4::TrajTransfer > {
  static const char* value()
  {
    return "bridge_px4/TrajTransfer";
  }

  static const char* value(const ::bridge_px4::TrajTransfer&) { return value(); }
};


// service_traits::MD5Sum< ::bridge_px4::TrajTransferRequest> should match 
// service_traits::MD5Sum< ::bridge_px4::TrajTransfer > 
template<>
struct MD5Sum< ::bridge_px4::TrajTransferRequest>
{
  static const char* value()
  {
    return MD5Sum< ::bridge_px4::TrajTransfer >::value();
  }
  static const char* value(const ::bridge_px4::TrajTransferRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::bridge_px4::TrajTransferRequest> should match 
// service_traits::DataType< ::bridge_px4::TrajTransfer > 
template<>
struct DataType< ::bridge_px4::TrajTransferRequest>
{
  static const char* value()
  {
    return DataType< ::bridge_px4::TrajTransfer >::value();
  }
  static const char* value(const ::bridge_px4::TrajTransferRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::bridge_px4::TrajTransferResponse> should match 
// service_traits::MD5Sum< ::bridge_px4::TrajTransfer > 
template<>
struct MD5Sum< ::bridge_px4::TrajTransferResponse>
{
  static const char* value()
  {
    return MD5Sum< ::bridge_px4::TrajTransfer >::value();
  }
  static const char* value(const ::bridge_px4::TrajTransferResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::bridge_px4::TrajTransferResponse> should match 
// service_traits::DataType< ::bridge_px4::TrajTransfer > 
template<>
struct DataType< ::bridge_px4::TrajTransferResponse>
{
  static const char* value()
  {
    return DataType< ::bridge_px4::TrajTransfer >::value();
  }
  static const char* value(const ::bridge_px4::TrajTransferResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // BRIDGE_PX4_MESSAGE_TRAJTRANSFER_H