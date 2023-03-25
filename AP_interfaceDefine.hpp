#ifndef AP_INTERFACEDEFINE_HPP
#define AP_INTERFACEDEFINE_HPP

#include "sysd_framework/common.hpp"
#include "sysd_framework/sysd_component.hpp"
#include "sysd_framework/sysd_service.hpp"
#include "sysd_framework/sysd_diagnosis.hpp"
//包含自定义的接口文件
#include "idl_generated/DmkPlannerMsgsPubSubTypes.h"
#include "idl_generated/EgoVehicleCanInfoMsgPubSubTypes.h"
#include "idl_generated/EgoVehicleInfoMsgPubSubTypes.h"
#include "idl_generated/GuidanceMsgsPubSubTypes.h"
#include "idl_generated/LocalizationMsgPubSubTypes.h"
#include "idl_generated/ObjectInfoArrayPubSubTypes.h"   //感知障碍物
#include "idl_generated/RouteDataMsgsPubSubTypes.h"
#include "idl_generated/RemoteVehicleInfoMsgPubSubTypes.h"
#include "idl_generated/LateralControlMsgPubSubTypes.h"
#include "idl_generated/LongitudinalControlMsgPubSubTypes.h"

// #include "idl_generated/MainSwitchButtonPubSubTypes.h"
// #include "idl_generated/FeatureButtonPubSubTypes.h"
// #include "idl_generated/RouteStationSelectPubSubTypes.h"
// #include "idl_generated/SwitchMapSignalPubSubTypes.h"

#include "idl_generated/PlanningStatusPubSubTypes.h"
#include "idl_generated/ActionDecisionMsgPubSubTypes.h"

namespace actionPlanning
{
    //AP接受pin
    struct APInputPin
    {
        //can信息和状态
        sysd::InputPinCache<EgoVehicleCanInfoMsg> *vehicleCanInfo_in = nullptr;
        sysd::InputPinCache<EgoVehicleCanStatusMsg> *vehicleCanStatus_in = nullptr;
        //自车信息
        sysd::InputPinCache<EgoVehicleInfoMsgs> *vehicleInfo_in = nullptr;
        //状态机模块
        sysd::InputPinCache<ActionPlannerInfo> *actionPlannerInfo_in = nullptr;
        //定位信息
        sysd::InputPinCache<Localization> *localization_in = nullptr;
        //感知信息
        sysd::InputPinCache<FusionData> *objFusionData_in = nullptr;
        sysd::InputPinCache<RemoteVehicleInfoMsg> *remoteVehicleInfo_in = nullptr;
        //控制模块信息
        sysd::InputPinCache<LongitudinalControlMsg> *longitudinalControlMsg_in = nullptr;
        sysd::InputPinCache<LateralControlMsg> *lateralControlMsg_in = nullptr;
    };

    //AP发送pin
    struct APOutputPin
    {
        //输出轨迹点列
        sysd::OutputPin<DiscretizedTrajectory> *discretizedTrajectory_out = nullptr;
        //输出自车速度状态
        sysd::OutputPin<EgoVehicleInfoMsg> *vehicleInfo_out = nullptr;
    };

    //AP接受信号
    struct APInputSignal
    {
        const EgoVehicleCanInfoMsg *vehicleCanInfo = nullptr;
        const EgoVehicleCanStatusMsg *vehicleCanStatus = nullptr;

        const EgoVehicleInfoMsgs *vehicleInfo = nullptr;

        const ActionPlannerInfo *actionPlannerInfo = nullptr;

        const Localization *localization = nullptr;

        const FusionData *objFusionData = nullptr;
        const RemoteVehicleInfoMsg *remoteVehicleInfo = nullptr;

        const LongitudinalControlMsg *longitudinalControlMsg = nullptr;
        const LateralControlMsg *lateralControlMsg = nullptr;
    }; 
}   //namespace actionPlanning


#endif  //AP_INTERFACEDEFINE_HPP