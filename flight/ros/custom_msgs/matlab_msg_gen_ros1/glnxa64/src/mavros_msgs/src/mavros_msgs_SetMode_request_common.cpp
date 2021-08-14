// Copyright 2020 The MathWorks, Inc.
// Common copy functions for mavros_msgs/SetModeRequest
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "mavros_msgs/SetMode.h"
#include "visibility_control.h"
#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
#endif
#ifndef FOUNDATION_MATLABDATA_API
typedef matlab::data::Array MDArray_T;
typedef matlab::data::ArrayFactory MDFactory_T;
#else
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::standalone::ClientArrayFactory MDFactory_T;
#endif
namespace mavros_msgs {
namespace matlabhelper {
namespace SetMode_Request {
  //----------------------------------------------------------------------------
  MAVROS_MSGS_EXPORT void copy_from_arr(mavros_msgs::SetMode::Request& msg, const matlab::data::StructArray arr) {
    try {
        //base_mode
        const matlab::data::TypedArray<uint8_t> base_mode_arr = arr[0]["base_mode"];
        msg.base_mode = base_mode_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'base_mode' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'base_mode' is wrong type; expected a uint8.");
    }
    try {
        //custom_mode
        const matlab::data::CharArray custom_mode_arr = arr[0]["custom_mode"];
        msg.custom_mode = custom_mode_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'custom_mode' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'custom_mode' is wrong type; expected a string.");
    }
  }
  MAVROS_MSGS_EXPORT int64_t get_requestId_from_arr(const matlab::data::StructArray arr) {
    // Get the request ID
    int64_t requestId = 0;
    try {
        // data
        const matlab::data::TypedArray<int64_t> data_arr = arr[0]["requestId"];
        requestId = data_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'requestId' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'requestId' is wrong type; expected a int64.");
    }
    return requestId;
  }
  //----------------------------------------------------------------------------
  MAVROS_MSGS_EXPORT MDArray_T get_arr(MDFactory_T& factory, const mavros_msgs::SetMode::Request& msg) {
    auto outArray = factory.createStructArray({1,1},{"MAV_MODE_PREFLIGHT","MAV_MODE_STABILIZE_DISARMED","MAV_MODE_STABILIZE_ARMED","MAV_MODE_MANUAL_DISARMED","MAV_MODE_MANUAL_ARMED","MAV_MODE_GUIDED_DISARMED","MAV_MODE_GUIDED_ARMED","MAV_MODE_AUTO_DISARMED","MAV_MODE_AUTO_ARMED","MAV_MODE_TEST_DISARMED","MAV_MODE_TEST_ARMED","base_mode","custom_mode"});
    // MAV_MODE_PREFLIGHT
    outArray[0]["MAV_MODE_PREFLIGHT"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_PREFLIGHT));
    // MAV_MODE_STABILIZE_DISARMED
    outArray[0]["MAV_MODE_STABILIZE_DISARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_STABILIZE_DISARMED));
    // MAV_MODE_STABILIZE_ARMED
    outArray[0]["MAV_MODE_STABILIZE_ARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_STABILIZE_ARMED));
    // MAV_MODE_MANUAL_DISARMED
    outArray[0]["MAV_MODE_MANUAL_DISARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_MANUAL_DISARMED));
    // MAV_MODE_MANUAL_ARMED
    outArray[0]["MAV_MODE_MANUAL_ARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_MANUAL_ARMED));
    // MAV_MODE_GUIDED_DISARMED
    outArray[0]["MAV_MODE_GUIDED_DISARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_GUIDED_DISARMED));
    // MAV_MODE_GUIDED_ARMED
    outArray[0]["MAV_MODE_GUIDED_ARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_GUIDED_ARMED));
    // MAV_MODE_AUTO_DISARMED
    outArray[0]["MAV_MODE_AUTO_DISARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_AUTO_DISARMED));
    // MAV_MODE_AUTO_ARMED
    outArray[0]["MAV_MODE_AUTO_ARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_AUTO_ARMED));
    // MAV_MODE_TEST_DISARMED
    outArray[0]["MAV_MODE_TEST_DISARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_TEST_DISARMED));
    // MAV_MODE_TEST_ARMED
    outArray[0]["MAV_MODE_TEST_ARMED"] = factory.createScalar(static_cast<uint8_t>(msg.MAV_MODE_TEST_ARMED));
    // base_mode
    outArray[0]["base_mode"] = factory.createScalar(msg.base_mode);
    // custom_mode
    outArray[0]["custom_mode"] = factory.createCharArray(msg.custom_mode);
    return std::move(outArray);
  }
} //namespace SetMode_Request
} //namespace matlabhelper
} //namespace mavros_msgs
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1