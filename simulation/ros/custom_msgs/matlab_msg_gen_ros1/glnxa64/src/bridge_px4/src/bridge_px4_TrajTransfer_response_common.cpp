// Copyright 2020 The MathWorks, Inc.
// Common copy functions of server for bridge_px4/TrajTransferResponse
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
#include "bridge_px4/TrajTransfer.h"
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
namespace bridge_px4 {
namespace matlabhelper {
namespace TrajTransfer_Response {
  //----------------------------------------------------------------------------
  BRIDGE_PX4_EXPORT void copy_from_arr(bridge_px4::TrajTransfer::Response& msg, const matlab::data::StructArray arr) {
    try {
        //checksum
        const matlab::data::TypedArray<bool> checksum_arr = arr[0]["checksum"];
        msg.checksum = checksum_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'checksum' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'checksum' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  BRIDGE_PX4_EXPORT MDArray_T get_arr(MDFactory_T& factory, const bridge_px4::TrajTransfer::Response& msg) {
    auto outArray = factory.createStructArray({1,1},{"checksum"});
    // checksum
    outArray[0]["checksum"] = factory.createScalar(static_cast<bool>(msg.checksum));
    return std::move(outArray);
  }
} //namespace TrajTransfer_Response
} //namespace matlabhelper
} //namespace bridge_px4
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1