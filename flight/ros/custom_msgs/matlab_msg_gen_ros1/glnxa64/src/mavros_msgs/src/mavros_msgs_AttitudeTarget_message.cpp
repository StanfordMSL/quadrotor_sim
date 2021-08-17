// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for mavros_msgs/AttitudeTarget
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
#include "mavros_msgs/AttitudeTarget.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class MAVROS_MSGS_EXPORT mavros_msgs_msg_AttitudeTarget_common : public MATLABROSMsgInterface<mavros_msgs::AttitudeTarget> {
  public:
    virtual ~mavros_msgs_msg_AttitudeTarget_common(){}
    virtual void copy_from_struct(mavros_msgs::AttitudeTarget* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const mavros_msgs::AttitudeTarget* msg, MultiLibLoader loader, size_t size = 1);
};
  void mavros_msgs_msg_AttitudeTarget_common::copy_from_struct(mavros_msgs::AttitudeTarget* msg, const matlab::data::Struct& arr,
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
        //type_mask
        const matlab::data::TypedArray<uint8_t> type_mask_arr = arr["TypeMask"];
        msg->type_mask = type_mask_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'TypeMask' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'TypeMask' is wrong type; expected a uint8.");
    }
    try {
        //orientation
        const matlab::data::StructArray orientation_arr = arr["Orientation"];
        static auto msgClassPtr_orientation = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Quaternion>>("geometry_msgs_msg_Quaternion_common");
        msgClassPtr_orientation->copy_from_struct(&msg->orientation,orientation_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Orientation' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Orientation' is wrong type; expected a struct.");
    }
    try {
        //body_rate
        const matlab::data::StructArray body_rate_arr = arr["BodyRate"];
        static auto msgClassPtr_body_rate = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Vector3>>("geometry_msgs_msg_Vector3_common");
        msgClassPtr_body_rate->copy_from_struct(&msg->body_rate,body_rate_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BodyRate' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BodyRate' is wrong type; expected a struct.");
    }
    try {
        //thrust
        const matlab::data::TypedArray<float> thrust_arr = arr["Thrust"];
        msg->thrust = thrust_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Thrust' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Thrust' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T mavros_msgs_msg_AttitudeTarget_common::get_arr(MDFactory_T& factory, const mavros_msgs::AttitudeTarget* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","TypeMask","IGNOREROLLRATE","IGNOREPITCHRATE","IGNOREYAWRATE","IGNORETHRUST","IGNOREATTITUDE","Orientation","BodyRate","Thrust"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("mavros_msgs/AttitudeTarget");
    // header
    auto currentElement_header = (msg + ctr)->header;
    static auto msgClassPtr_header = loader->createInstance<MATLABROSMsgInterface<std_msgs::Header>>("std_msgs_msg_Header_common");
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // type_mask
    auto currentElement_type_mask = (msg + ctr)->type_mask;
    outArray[ctr]["TypeMask"] = factory.createScalar(currentElement_type_mask);
    // IGNORE_ROLL_RATE
    auto currentElement_IGNORE_ROLL_RATE = (msg + ctr)->IGNORE_ROLL_RATE;
    outArray[ctr]["IGNOREROLLRATE"] = factory.createScalar(static_cast<uint8_t>(currentElement_IGNORE_ROLL_RATE));
    // IGNORE_PITCH_RATE
    auto currentElement_IGNORE_PITCH_RATE = (msg + ctr)->IGNORE_PITCH_RATE;
    outArray[ctr]["IGNOREPITCHRATE"] = factory.createScalar(static_cast<uint8_t>(currentElement_IGNORE_PITCH_RATE));
    // IGNORE_YAW_RATE
    auto currentElement_IGNORE_YAW_RATE = (msg + ctr)->IGNORE_YAW_RATE;
    outArray[ctr]["IGNOREYAWRATE"] = factory.createScalar(static_cast<uint8_t>(currentElement_IGNORE_YAW_RATE));
    // IGNORE_THRUST
    auto currentElement_IGNORE_THRUST = (msg + ctr)->IGNORE_THRUST;
    outArray[ctr]["IGNORETHRUST"] = factory.createScalar(static_cast<uint8_t>(currentElement_IGNORE_THRUST));
    // IGNORE_ATTITUDE
    auto currentElement_IGNORE_ATTITUDE = (msg + ctr)->IGNORE_ATTITUDE;
    outArray[ctr]["IGNOREATTITUDE"] = factory.createScalar(static_cast<uint8_t>(currentElement_IGNORE_ATTITUDE));
    // orientation
    auto currentElement_orientation = (msg + ctr)->orientation;
    static auto msgClassPtr_orientation = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Quaternion>>("geometry_msgs_msg_Quaternion_common");
    outArray[ctr]["Orientation"] = msgClassPtr_orientation->get_arr(factory, &currentElement_orientation, loader);
    // body_rate
    auto currentElement_body_rate = (msg + ctr)->body_rate;
    static auto msgClassPtr_body_rate = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Vector3>>("geometry_msgs_msg_Vector3_common");
    outArray[ctr]["BodyRate"] = msgClassPtr_body_rate->get_arr(factory, &currentElement_body_rate, loader);
    // thrust
    auto currentElement_thrust = (msg + ctr)->thrust;
    outArray[ctr]["Thrust"] = factory.createScalar(currentElement_thrust);
    }
    return std::move(outArray);
  } 
class MAVROS_MSGS_EXPORT mavros_msgs_AttitudeTarget_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~mavros_msgs_AttitudeTarget_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          mavros_msgs_AttitudeTarget_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<mavros_msgs::AttitudeTarget,mavros_msgs_msg_AttitudeTarget_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         mavros_msgs_AttitudeTarget_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<mavros_msgs::AttitudeTarget,mavros_msgs::AttitudeTarget::ConstPtr,mavros_msgs_msg_AttitudeTarget_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_msg_AttitudeTarget_common, MATLABROSMsgInterface<mavros_msgs::AttitudeTarget>)
CLASS_LOADER_REGISTER_CLASS(mavros_msgs_AttitudeTarget_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1