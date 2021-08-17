// Copyright 2019-2020 The MathWorks, Inc.
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
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class BRIDGE_PX4_EXPORT bridge_px4_msg_TrajTransferRequest_common : public MATLABROSMsgInterface<bridge_px4::TrajTransfer::Request> {
  public:
    virtual ~bridge_px4_msg_TrajTransferRequest_common(){}
    virtual void copy_from_struct(bridge_px4::TrajTransfer::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const bridge_px4::TrajTransfer::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void bridge_px4_msg_TrajTransferRequest_common::copy_from_struct(bridge_px4::TrajTransfer::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //hz
        const matlab::data::TypedArray<int32_t> hz_arr = arr["Hz"];
        msg->hz = hz_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Hz' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Hz' is wrong type; expected a int32.");
    }
    try {
        //N
        const matlab::data::TypedArray<int32_t> N_arr = arr["N"];
        msg->N = N_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'N' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'N' is wrong type; expected a int32.");
    }
    try {
        //u_arr
        const matlab::data::TypedArray<float> u_arr_arr = arr["UArr"];
        size_t nelem = u_arr_arr.getNumberOfElements();
        	msg->u_arr.resize(nelem);
        	std::copy(u_arr_arr.begin(), u_arr_arr.begin()+nelem, msg->u_arr.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'UArr' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'UArr' is wrong type; expected a single.");
    }
    try {
        //x_arr
        const matlab::data::TypedArray<float> x_arr_arr = arr["XArr"];
        size_t nelem = x_arr_arr.getNumberOfElements();
        	msg->x_arr.resize(nelem);
        	std::copy(x_arr_arr.begin(), x_arr_arr.begin()+nelem, msg->x_arr.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'XArr' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'XArr' is wrong type; expected a single.");
    }
    try {
        //L_arr
        const matlab::data::TypedArray<float> L_arr_arr = arr["LArr"];
        size_t nelem = L_arr_arr.getNumberOfElements();
        	msg->L_arr.resize(nelem);
        	std::copy(L_arr_arr.begin(), L_arr_arr.begin()+nelem, msg->L_arr.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'LArr' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'LArr' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T bridge_px4_msg_TrajTransferRequest_common::get_arr(MDFactory_T& factory, const bridge_px4::TrajTransfer::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Hz","N","UArr","XArr","LArr"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("bridge_px4/TrajTransferRequest");
    // hz
    auto currentElement_hz = (msg + ctr)->hz;
    outArray[ctr]["Hz"] = factory.createScalar(currentElement_hz);
    // N
    auto currentElement_N = (msg + ctr)->N;
    outArray[ctr]["N"] = factory.createScalar(currentElement_N);
    // u_arr
    auto currentElement_u_arr = (msg + ctr)->u_arr;
    outArray[ctr]["UArr"] = factory.createArray<bridge_px4::TrajTransfer::Request::_u_arr_type::const_iterator, float>({currentElement_u_arr.size(),1}, currentElement_u_arr.begin(), currentElement_u_arr.end());
    // x_arr
    auto currentElement_x_arr = (msg + ctr)->x_arr;
    outArray[ctr]["XArr"] = factory.createArray<bridge_px4::TrajTransfer::Request::_x_arr_type::const_iterator, float>({currentElement_x_arr.size(),1}, currentElement_x_arr.begin(), currentElement_x_arr.end());
    // L_arr
    auto currentElement_L_arr = (msg + ctr)->L_arr;
    outArray[ctr]["LArr"] = factory.createArray<bridge_px4::TrajTransfer::Request::_L_arr_type::const_iterator, float>({currentElement_L_arr.size(),1}, currentElement_L_arr.begin(), currentElement_L_arr.end());
    }
    return std::move(outArray);
  }
class BRIDGE_PX4_EXPORT bridge_px4_msg_TrajTransferResponse_common : public MATLABROSMsgInterface<bridge_px4::TrajTransfer::Response> {
  public:
    virtual ~bridge_px4_msg_TrajTransferResponse_common(){}
    virtual void copy_from_struct(bridge_px4::TrajTransfer::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const bridge_px4::TrajTransfer::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void bridge_px4_msg_TrajTransferResponse_common::copy_from_struct(bridge_px4::TrajTransfer::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //checksum
        const matlab::data::TypedArray<bool> checksum_arr = arr["Checksum"];
        msg->checksum = checksum_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Checksum' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Checksum' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T bridge_px4_msg_TrajTransferResponse_common::get_arr(MDFactory_T& factory, const bridge_px4::TrajTransfer::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Checksum"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("bridge_px4/TrajTransferResponse");
    // checksum
    auto currentElement_checksum = (msg + ctr)->checksum;
    outArray[ctr]["Checksum"] = factory.createScalar(static_cast<bool>(currentElement_checksum));
    }
    return std::move(outArray);
  } 
class BRIDGE_PX4_EXPORT bridge_px4_TrajTransfer_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~bridge_px4_TrajTransfer_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          bridge_px4_TrajTransfer_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<bridge_px4::TrajTransfer::Request,bridge_px4_msg_TrajTransferRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<bridge_px4::TrajTransfer::Response,bridge_px4_msg_TrajTransferResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          bridge_px4_TrajTransfer_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<bridge_px4::TrajTransfer::Request,bridge_px4::TrajTransfer::Request::ConstPtr,bridge_px4_msg_TrajTransferRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<bridge_px4::TrajTransfer::Response,bridge_px4::TrajTransfer::Response::ConstPtr,bridge_px4_msg_TrajTransferResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          bridge_px4_TrajTransfer_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<bridge_px4::TrajTransfer::Request,bridge_px4::TrajTransfer::Response,bridge_px4_msg_TrajTransferRequest_common,bridge_px4_msg_TrajTransferResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          bridge_px4_TrajTransfer_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<bridge_px4::TrajTransfer,bridge_px4::TrajTransfer::Request,bridge_px4::TrajTransfer::Response,bridge_px4_msg_TrajTransferRequest_common,bridge_px4_msg_TrajTransferResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(bridge_px4_msg_TrajTransferRequest_common, MATLABROSMsgInterface<bridge_px4::TrajTransfer::Request>)
CLASS_LOADER_REGISTER_CLASS(bridge_px4_msg_TrajTransferResponse_common, MATLABROSMsgInterface<bridge_px4::TrajTransfer::Response>)
CLASS_LOADER_REGISTER_CLASS(bridge_px4_TrajTransfer_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
