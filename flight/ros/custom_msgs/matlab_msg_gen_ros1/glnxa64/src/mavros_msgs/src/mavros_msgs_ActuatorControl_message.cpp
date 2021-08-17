// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for mavros_msgs/ActuatorControl
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
#include "mavros_msgs/ActuatorControl.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class MAVROS_MSGS_EXPORT mavros_msgs_msg_ActuatorControl_common : public MATLABROSMsgInterface<mavros_msgs::ActuatorControl> {
  public:
    virtual ~mavros_msgs_msg_ActuatorControl_common(){}
    virtual void copy_from_struct(mavros_msgs::ActuatorControl* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const mavros_msgs::ActuatorControl* msg, MultiLibLoader loader, size_t size = 1);
};
  void mavros_msgs_msg_ActuatorControl_common::copy_from_struct(mavros_msgs::ActuatorControl* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["Header"];
        static auto msgClassPtr_header = loader->createInstance<MATLABROSMsgInterface<std_msgs::Header>>("std_msgs_msg_Header_common");
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Header' is wrong type; expected a struct.");
    }
    try {
        //group_mix
        const matlab::data::TypedArray<uint8_t> group_mix_arr = arr["GroupMix"];
        msg->group_mix = group_mix_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'GroupMix' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'GroupMix' is wrong type; expected a uint8.");
    }
    try {
        //controls
        const matlab::data::TypedArray<float> controls_arr = arr["Controls"];
        size_t nelem = 8;
        	std::copy(controls_arr.begin(), controls_arr.begin()+nelem, msg->controls.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Controls' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Controls' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T mavros_msgs_msg_ActuatorControl_common::get_arr(MDFactory_T& factory, const mavros_msgs::ActuatorControl* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","PX4MIXFLIGHTCONTROL","PX4MIXFLIGHTCONTROLVTOLALT","PX4MIXPAYLOAD","PX4MIXMANUALPASSTHROUGH","Header","GroupMix","Controls"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("mavros_msgs/ActuatorControl");
    // PX4_MIX_FLIGHT_CONTROL
    auto currentElement_PX4_MIX_FLIGHT_CONTROL = (msg + ctr)->PX4_MIX_FLIGHT_CONTROL;
    outArray[ctr]["PX4MIXFLIGHTCONTROL"] = factory.createScalar(static_cast<uint8_t>(currentElement_PX4_MIX_FLIGHT_CONTROL));
    // PX4_MIX_FLIGHT_CONTROL_VTOL_ALT
    auto currentElement_PX4_MIX_FLIGHT_CONTROL_VTOL_ALT = (msg + ctr)->PX4_MIX_FLIGHT_CONTROL_VTOL_ALT;
    outArray[ctr]["PX4MIXFLIGHTCONTROLVTOLALT"] = factory.createScalar(static_cast<uint8_t>(currentElement_PX4_MIX_FLIGHT_CONTROL_VTOL_ALT));
    // PX4_MIX_PAYLOAD
    auto currentElement_PX4_MIX_PAYLOAD = (msg + ctr)->PX4_MIX_PAYLOAD;
    outArray[ctr]["PX4MIXPAYLOAD"] = factory.createScalar(static_cast<uint8_t>(currentElement_PX4_MIX_PAYLOAD));
    // PX4_MIX_MANUAL_PASSTHROUGH
    auto currentElement_PX4_MIX_MANUAL_PASSTHROUGH = (msg + ctr)->PX4_MIX_MANUAL_PASSTHROUGH;
    outArray[ctr]["PX4MIXMANUALPASSTHROUGH"] = factory.createScalar(static_cast<uint8_t>(currentElement_PX4_MIX_MANUAL_PASSTHROUGH));
    // header
    auto currentElement_header = (msg + ctr)->header;
    static auto msgClassPtr_header = loader->createInstance<MATLABROSMsgInterface<std_msgs::Header>>("std_msgs_msg_Header_common");
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // group_mix
    auto currentElement_group_mix = (msg + ctr)->group_mix;
    outArray[ctr]["GroupMix"] = factory.createScalar(currentElement_group_mix);
    // controls
    auto currentElement_controls = (msg + ctr)->controls;
    outArray[ctr]["Controls"] = factory.createArray<mavros_msgs::ActuatorControl::_controls_type::const_iterator, float>({currentElement_controls.size(),1}, currentElement_controls.begin(), currentElement_controls.end());
    }
    return std::move(outArray);
  } 
class MAVROS_MSGS_EXPORT mavros_msgs_ActuatorControl_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~mavros_msgs_ActuatorControl_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          mavros_msgs_ActuatorControl_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<mavros_msgs::ActuatorControl,mavros_msgs_msg_ActuatorControl_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         mavros_msgs_ActuatorControl_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<mavros_msgs::ActuatorControl,mavros_msgs::ActuatorControl::ConstPtr,mavros_msgs_msg_ActuatorControl_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_msg_ActuatorControl_common, MATLABROSMsgInterface<mavros_msgs::ActuatorControl>)
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_ActuatorControl_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1