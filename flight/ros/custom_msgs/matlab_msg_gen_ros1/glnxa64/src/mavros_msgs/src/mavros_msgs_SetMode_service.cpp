// Copyright 2019-2020 The MathWorks, Inc.
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
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class MAVROS_MSGS_EXPORT mavros_msgs_msg_SetModeRequest_common : public MATLABROSMsgInterface<mavros_msgs::SetMode::Request> {
  public:
    virtual ~mavros_msgs_msg_SetModeRequest_common(){}
    virtual void copy_from_struct(mavros_msgs::SetMode::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const mavros_msgs::SetMode::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void mavros_msgs_msg_SetModeRequest_common::copy_from_struct(mavros_msgs::SetMode::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //base_mode
        const matlab::data::TypedArray<uint8_t> base_mode_arr = arr["BaseMode"];
        msg->base_mode = base_mode_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BaseMode' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BaseMode' is wrong type; expected a uint8.");
    }
    try {
        //custom_mode
        const matlab::data::CharArray custom_mode_arr = arr["CustomMode"];
        msg->custom_mode = custom_mode_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'CustomMode' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'CustomMode' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T mavros_msgs_msg_SetModeRequest_common::get_arr(MDFactory_T& factory, const mavros_msgs::SetMode::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","MAVMODEPREFLIGHT","MAVMODESTABILIZEDISARMED","MAVMODESTABILIZEARMED","MAVMODEMANUALDISARMED","MAVMODEMANUALARMED","MAVMODEGUIDEDDISARMED","MAVMODEGUIDEDARMED","MAVMODEAUTODISARMED","MAVMODEAUTOARMED","MAVMODETESTDISARMED","MAVMODETESTARMED","BaseMode","CustomMode"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("mavros_msgs/SetModeRequest");
    // MAV_MODE_PREFLIGHT
    auto currentElement_MAV_MODE_PREFLIGHT = (msg + ctr)->MAV_MODE_PREFLIGHT;
    outArray[ctr]["MAVMODEPREFLIGHT"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_PREFLIGHT));
    // MAV_MODE_STABILIZE_DISARMED
    auto currentElement_MAV_MODE_STABILIZE_DISARMED = (msg + ctr)->MAV_MODE_STABILIZE_DISARMED;
    outArray[ctr]["MAVMODESTABILIZEDISARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_STABILIZE_DISARMED));
    // MAV_MODE_STABILIZE_ARMED
    auto currentElement_MAV_MODE_STABILIZE_ARMED = (msg + ctr)->MAV_MODE_STABILIZE_ARMED;
    outArray[ctr]["MAVMODESTABILIZEARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_STABILIZE_ARMED));
    // MAV_MODE_MANUAL_DISARMED
    auto currentElement_MAV_MODE_MANUAL_DISARMED = (msg + ctr)->MAV_MODE_MANUAL_DISARMED;
    outArray[ctr]["MAVMODEMANUALDISARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_MANUAL_DISARMED));
    // MAV_MODE_MANUAL_ARMED
    auto currentElement_MAV_MODE_MANUAL_ARMED = (msg + ctr)->MAV_MODE_MANUAL_ARMED;
    outArray[ctr]["MAVMODEMANUALARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_MANUAL_ARMED));
    // MAV_MODE_GUIDED_DISARMED
    auto currentElement_MAV_MODE_GUIDED_DISARMED = (msg + ctr)->MAV_MODE_GUIDED_DISARMED;
    outArray[ctr]["MAVMODEGUIDEDDISARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_GUIDED_DISARMED));
    // MAV_MODE_GUIDED_ARMED
    auto currentElement_MAV_MODE_GUIDED_ARMED = (msg + ctr)->MAV_MODE_GUIDED_ARMED;
    outArray[ctr]["MAVMODEGUIDEDARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_GUIDED_ARMED));
    // MAV_MODE_AUTO_DISARMED
    auto currentElement_MAV_MODE_AUTO_DISARMED = (msg + ctr)->MAV_MODE_AUTO_DISARMED;
    outArray[ctr]["MAVMODEAUTODISARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_AUTO_DISARMED));
    // MAV_MODE_AUTO_ARMED
    auto currentElement_MAV_MODE_AUTO_ARMED = (msg + ctr)->MAV_MODE_AUTO_ARMED;
    outArray[ctr]["MAVMODEAUTOARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_AUTO_ARMED));
    // MAV_MODE_TEST_DISARMED
    auto currentElement_MAV_MODE_TEST_DISARMED = (msg + ctr)->MAV_MODE_TEST_DISARMED;
    outArray[ctr]["MAVMODETESTDISARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_TEST_DISARMED));
    // MAV_MODE_TEST_ARMED
    auto currentElement_MAV_MODE_TEST_ARMED = (msg + ctr)->MAV_MODE_TEST_ARMED;
    outArray[ctr]["MAVMODETESTARMED"] = factory.createScalar(static_cast<uint8_t>(currentElement_MAV_MODE_TEST_ARMED));
    // base_mode
    auto currentElement_base_mode = (msg + ctr)->base_mode;
    outArray[ctr]["BaseMode"] = factory.createScalar(currentElement_base_mode);
    // custom_mode
    auto currentElement_custom_mode = (msg + ctr)->custom_mode;
    outArray[ctr]["CustomMode"] = factory.createCharArray(currentElement_custom_mode);
    }
    return std::move(outArray);
  }
class MAVROS_MSGS_EXPORT mavros_msgs_msg_SetModeResponse_common : public MATLABROSMsgInterface<mavros_msgs::SetMode::Response> {
  public:
    virtual ~mavros_msgs_msg_SetModeResponse_common(){}
    virtual void copy_from_struct(mavros_msgs::SetMode::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const mavros_msgs::SetMode::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void mavros_msgs_msg_SetModeResponse_common::copy_from_struct(mavros_msgs::SetMode::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //mode_sent
        const matlab::data::TypedArray<bool> mode_sent_arr = arr["ModeSent"];
        msg->mode_sent = mode_sent_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ModeSent' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ModeSent' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T mavros_msgs_msg_SetModeResponse_common::get_arr(MDFactory_T& factory, const mavros_msgs::SetMode::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ModeSent"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("mavros_msgs/SetModeResponse");
    // mode_sent
    auto currentElement_mode_sent = (msg + ctr)->mode_sent;
    outArray[ctr]["ModeSent"] = factory.createScalar(static_cast<bool>(currentElement_mode_sent));
    }
    return std::move(outArray);
  } 
class MAVROS_MSGS_EXPORT mavros_msgs_SetMode_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~mavros_msgs_SetMode_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          mavros_msgs_SetMode_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<mavros_msgs::SetMode::Request,mavros_msgs_msg_SetModeRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<mavros_msgs::SetMode::Response,mavros_msgs_msg_SetModeResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          mavros_msgs_SetMode_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<mavros_msgs::SetMode::Request,mavros_msgs::SetMode::Request::ConstPtr,mavros_msgs_msg_SetModeRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<mavros_msgs::SetMode::Response,mavros_msgs::SetMode::Response::ConstPtr,mavros_msgs_msg_SetModeResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          mavros_msgs_SetMode_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<mavros_msgs::SetMode::Request,mavros_msgs::SetMode::Response,mavros_msgs_msg_SetModeRequest_common,mavros_msgs_msg_SetModeResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          mavros_msgs_SetMode_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<mavros_msgs::SetMode,mavros_msgs::SetMode::Request,mavros_msgs::SetMode::Response,mavros_msgs_msg_SetModeRequest_common,mavros_msgs_msg_SetModeResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_msg_SetModeRequest_common, MATLABROSMsgInterface<mavros_msgs::SetMode::Request>)
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_msg_SetModeResponse_common, MATLABROSMsgInterface<mavros_msgs::SetMode::Response>)
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_SetMode_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
