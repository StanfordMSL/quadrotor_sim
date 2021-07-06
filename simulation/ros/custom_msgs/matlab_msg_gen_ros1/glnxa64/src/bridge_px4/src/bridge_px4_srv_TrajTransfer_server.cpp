// Copyright 2020 The MathWorks, Inc.
// Service Server for bridge_px4/TrajTransferResponse
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
#include "MATLABSvcServerInterface.hpp"
#include "visibility_control.h"
#include "bridge_px4/TrajTransfer.h"
#include <thread>
namespace bridge_px4 {
namespace matlabhelper {
namespace TrajTransfer_Request {
BRIDGE_PX4_EXPORT MDArray_T get_arr(MDFactory_T& factory,
                                         const bridge_px4::TrajTransfer::Request& request);
} // namespace TrajTransfer_Request
} // namespace matlabhelper
} // namespace bridge_px4
namespace bridge_px4 {
namespace matlabhelper {
namespace TrajTransfer_Response {
BRIDGE_PX4_EXPORT void copy_from_arr(bridge_px4::TrajTransfer::Response& response,
                                          const matlab::data::StructArray arr);
} // namespace TrajTransfer_Response
} // namespace matlabhelper
} // namespace bridge_px4
class BRIDGE_PX4_EXPORT bridge_px4_srv_TrajTransfer_server
    : public MATLABSvcServerInterface {
    bridge_px4::TrajTransfer::Response mResponse;
    void* mSd;
    SendDataToMATLABFunc_T mSendDataToMATLABFunc;
    intptr_t mHSvcServer;
    bool callback(bridge_px4::TrajTransfer::Request& req,
                  bridge_px4::TrajTransfer::Response& res) {
        auto outArray = bridge_px4::matlabhelper::TrajTransfer_Request::get_arr(mFactory, req);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mSendDataToMATLABFunc, outArray, mHSvcServer);
        }
        // After receiving the response, check if any error occurred
        // and in case of error, return false to service-client.
        if (getCallbackError()) {
            return false;
        }
        // If everything is fine, send the received response.
        res = mResponse;
        return true;
    }
  public:
    bridge_px4_srv_TrajTransfer_server()
        : MATLABSvcServerInterface()
        , mResponse()
        , mSd(NULL)
        , mSendDataToMATLABFunc()
        , mHSvcServer(0) {
    }
    virtual ~bridge_px4_srv_TrajTransfer_server() {
    }
    virtual bool setResponseFromMatlab(matlab::data::Array arr) {
        bridge_px4::matlabhelper::TrajTransfer_Response::copy_from_arr(mResponse, arr);
        return true;
    }
    virtual intptr_t createSvcServer(const std::string& svc_name,
                                     std::shared_ptr<ros::NodeHandle> theNode,
                                     void* sd,
                                     SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                     intptr_t hSvcServer) {
        mSd = sd;
        mSendDataToMATLABFunc = sendDataToMATLABFunc;
        mHSvcServer = hSvcServer;
        mSvc = std::make_shared<ros::ServiceServer>(theNode->advertiseService(
            svc_name, &bridge_px4_srv_TrajTransfer_server::callback, this));
        return reinterpret_cast<intptr_t>(mSvc.get());
    }
};
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(bridge_px4_srv_TrajTransfer_server, MATLABSvcServerInterface)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif
