// Copyright 2020 The MathWorks, Inc.
// Common copy functions for bridge_px4/TrajTransferRequest
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
namespace TrajTransfer_Request {
  //----------------------------------------------------------------------------
  BRIDGE_PX4_EXPORT void copy_from_arr(bridge_px4::TrajTransfer::Request& msg, const matlab::data::StructArray arr) {
    try {
        //hz
        const matlab::data::TypedArray<int32_t> hz_arr = arr[0]["hz"];
        msg.hz = hz_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'hz' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'hz' is wrong type; expected a int32.");
    }
    try {
        //N
        const matlab::data::TypedArray<int32_t> N_arr = arr[0]["N"];
        msg.N = N_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'N' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'N' is wrong type; expected a int32.");
    }
    try {
        //u_arr
        const matlab::data::TypedArray<float> u_arr_arr = arr[0]["u_arr"];
        size_t nelem = u_arr_arr.getNumberOfElements();
        	msg.u_arr.resize(nelem);
        	std::copy(u_arr_arr.begin(), u_arr_arr.begin()+nelem, msg.u_arr.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'u_arr' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'u_arr' is wrong type; expected a single.");
    }
    try {
        //L_arr
        const matlab::data::TypedArray<float> L_arr_arr = arr[0]["L_arr"];
        size_t nelem = L_arr_arr.getNumberOfElements();
        	msg.L_arr.resize(nelem);
        	std::copy(L_arr_arr.begin(), L_arr_arr.begin()+nelem, msg.L_arr.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'L_arr' is missing.");
    } catch (matlab::data::TypeMismatchException&) {
        throw std::invalid_argument("Field 'L_arr' is wrong type; expected a single.");
    }
  }
  BRIDGE_PX4_EXPORT int64_t get_requestId_from_arr(const matlab::data::StructArray arr) {
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
  BRIDGE_PX4_EXPORT MDArray_T get_arr(MDFactory_T& factory, const bridge_px4::TrajTransfer::Request& msg) {
    auto outArray = factory.createStructArray({1,1},{"hz","N","u_arr","L_arr"});
    // hz
    outArray[0]["hz"] = factory.createScalar(msg.hz);
    // N
    outArray[0]["N"] = factory.createScalar(msg.N);
    // u_arr
    outArray[0]["u_arr"] = factory.createArray<bridge_px4::TrajTransfer::Request::_u_arr_type::const_iterator, float>({1, msg.u_arr.size()}, msg.u_arr.begin(), msg.u_arr.end());
    // L_arr
    outArray[0]["L_arr"] = factory.createArray<bridge_px4::TrajTransfer::Request::_L_arr_type::const_iterator, float>({1, msg.L_arr.size()}, msg.L_arr.begin(), msg.L_arr.end());
    return std::move(outArray);
  }
} //namespace TrajTransfer_Request
} //namespace matlabhelper
} //namespace bridge_px4
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1