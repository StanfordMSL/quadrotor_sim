// Copyright 2020 The MathWorks, Inc.
// Service Client for mavros_msgs/SetModeRequest
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
#include "MATLABSvcClientInterface.hpp"
#include "visibility_control.h"
#include "mavros_msgs/SetMode.h"
#include <thread>
namespace mavros_msgs {
namespace matlabhelper {
namespace SetMode_Request {
MAVROS_MSGS_EXPORT void copy_from_arr(mavros_msgs::SetMode::Request& request,
                                          const matlab::data::StructArray arr);
MAVROS_MSGS_EXPORT int64_t get_requestId_from_arr(const matlab::data::StructArray arr);
} // namespace SetMode_Request
} // namespace matlabhelper
} // namespace mavros_msgs
namespace mavros_msgs {
namespace matlabhelper {
namespace SetMode_Response {
MAVROS_MSGS_EXPORT MDArray_T get_arr(MDFactory_T& factory,
                                         const mavros_msgs::SetMode::Response& response);
} // namespace SetMode_Response
} // namespace matlabhelper
} // namespace mavros_msgs
class MAVROS_MSGS_EXPORT mavros_msgs_srv_SetMode_client
    : public MATLABSvcClientInterface {
    // mRequestPool is moved from SvcClientData to here so that we can throw the user field-error
    // exception in MATLAB console.
    std::map<int64_t, mavros_msgs::SetMode::Request> mRequestPool;
    mavros_msgs::SetMode::Request mNextRequest;
  public:
    mavros_msgs_srv_SetMode_client()
        : MATLABSvcClientInterface()
        , mNextRequest() {
    }
    virtual ~mavros_msgs_srv_SetMode_client() {
    }
    virtual intptr_t createSvcClient(const std::string& srv_name,
                                     std::shared_ptr<ros::NodeHandle> theNode) {
        mClient = std::make_shared<ros::ServiceClient>(
            theNode->serviceClient<mavros_msgs::SetMode>(srv_name));
        return reinterpret_cast<intptr_t>(mClient.get());
    }
    virtual bool send_request(void* sd,
                              SendDataToMATLABFunc_T sendDataToMATLABFunc,
                              const intptr_t hSvcClient) {
        mavros_msgs::SetMode svc;
        svc.request = mNextRequest;
        // Just before processing the request, double Check if timeout has
        // occurred from MATLAB or not. In-case of timeout, no need to send the
        // request for processing.
        if (getNextRequestTimeoutState()) {
            return false;
        }
        // send the request for processing
        auto ret = mClient->call(svc);
        // After the request is processed, Check if timeout has
        // occurred from MATLAB in between. In-case of timeout, no need to send
        // the response to MATLAB.
        if (getNextRequestTimeoutState()) {
            return false;
        }
        if (ret) {
            // send the result back
            auto outArray =
                mavros_msgs::matlabhelper::SetMode_Response::get_arr(mFactory, svc.response);
            if (sd != NULL) {
                appendAndSendToMATLAB(sd, sendDataToMATLABFunc, outArray, hSvcClient,
                                      mNextRequestId);
            } else {
                return false;
            }
        } else {
            // If server is not able to send the response,
            // send error message to MATLAB.
            auto outArray = mFactory.createStructArray({1, 1}, {"serverCallbackError"});
            outArray[0]["serverCallbackError"] =
                mFactory.createCharArray("Failed to get response from service-server.");
            if (sd != NULL) {
                appendAndSendToMATLAB(sd, sendDataToMATLABFunc, outArray, hSvcClient,
                                      mNextRequestId);
            }
            return false;
        }
        return true;
    }
    virtual bool setNextRequest() {
        if (mRequestPool.size() > 0) {
            auto it = mRequestPool.begin();
            mNextRequestId = it->first;
            mNextRequest = it->second;
            std::cout << "\nNext request-id : " << mNextRequestId;
            // delete from map after fetching the request
            mRequestPool.erase(mNextRequestId);
            // set timeout state false before processing request
            setNextRequestTimeoutState(false);
            return true;
        }
        // if no request left, return false.
        return false;
    }
    virtual bool addRequestToPool(matlab::data::Array arr) {
        mavros_msgs::SetMode::Request req;
        mavros_msgs::matlabhelper::SetMode_Request::copy_from_arr(req, arr);
        mRequestPool.insert(
                {mavros_msgs::matlabhelper::SetMode_Request::get_requestId_from_arr(arr), 
                         req});
        return true;
    }
    virtual bool clientRequestTimeout(int64_t requestId) {
        if (requestId == mNextRequestId) {
            // Check if the request is ongoing request, set the timeout
            // status true so that when response comes, it will be ignored
            // and not sent to MATLAB callback.
            setNextRequestTimeoutState(true);
        } else {
            // If the request is present in request-pool, it has not been
            // processed yet and it is waiting for the earlier request to
            // be processed. As already MATLAB timeout happened, so no need
            // to send to service-server any more and it will be removed.
            if (mRequestPool.find(requestId) != mRequestPool.end()) {
                mRequestPool.erase(requestId);
            } else {
                return false;
            }
        }
        return true;
    }
};
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_srv_SetMode_client, MATLABSvcClientInterface)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
