#ifndef AP_INTERPROCESS_HPP
#define AP_INTERPROCESS_HPP

#include "sysd_framework/sysd_pinTemplates.hpp"
#include "./AP_interfaceDefine.hpp"

namespace actionPlanning
{
    static int count_t = 0;
    static int count_t_canStatus = 0;

    inline bool getNewestSignals(
        APInputPin &inputPin,
        APInputSignal &inputSignal,
        sysd::Time      timeout,
        sysd::Time      *oldestTimeStamp = nullptr)
    {
        if(false == sysdGetNewestSignalTemplate(
            inputSignal.vehicleCanInfo,
            inputPin.vehicleCanInfo_in, timeout, oldestTimeStamp))
        {
            count_t ++;
            if(count_t > 10)
            {
                sysdPrintError("actionPlanning: vehicleCanInfo is null or too old!\n");
                return false; 
            }
        }
        else { count_t = 0; }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.vehicleCanStatus,
            inputPin.vehicleCanStatus_in, timeout, oldestTimeStamp))
        {
            count_t_canStatus ++;
            if(count_t_canStatus > 10)
            {
                sysdPrintError("actionPlanning: vehicleCanStatus is null or too old!\n");
                return false;
            }
        }
        else { count_t_canStatus = 0; }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.vehicleInfo,
            inputPin.vehicleInfo_in, timeout, oldestTimeStamp))
        {
            sysdPrintError("actionPlanning: vehicleInfo is null or too old!\n");
            return false;
        }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.actionPlannerInfo,
            inputPin.actionPlannerInfo_in, timeout, oldestTimeStamp))
        {
            sysdPrintError("actionPlaning: actionPlannerInfo is null or too old!\n");
            return false;
        }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.localization,
            inputPin.localization_in, timeout, oldestTimeStamp))
        {
            sysdPrintError("actionPlanning: localization is null or too old!\n");
            return false;
        }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.objFusionData,
            inputPin.objFusionData_in, timeout, oldestTimeStamp))
        {
            sysdPrintError("actionPlanning: FusionData is null or too old!\n");
            return false;
        }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.remoteVehicleInfo,
            inputPin.remoteVehicleInfo_in, timeout, oldestTimeStamp))
        {
            sysdPrintError("actionPlanning: remoteVehicleInfo is null or too old!\n");
            return false;
        }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.longitudinalControlMsg,
            inputPin.longitudinalControlMsg_in, timeout, oldestTimeStamp))
        {
            sysdPrintError("actionPlanning: longitudinalControl is null or too old!\n");
            return false;
        }

        if(false == sysdGetNewestSignalTemplate(
            inputSignal.lateralControlMsg,
            inputPin.lateralControlMsg_in, timeout, oldestTimeStamp))
        {
            sysdPrintError("actionPlanning: lateralControl is null or too old!\n");
            return false;
        }

        return true;
    }

    inline bool syncInputSignals(
        APInputPin &inputPin,
        APInputSignal &inputSignal,
        sysd::Time syncTime,
        sysd::Time syncTimeGap)
    {
        const EgoVehicleCanInfoMsg *vehicleCanInfo_tmp = nullptr;
        vehicleCanInfo_tmp = inputPin.vehicleCanInfo_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == vehicleCanInfo_tmp)
        {
            count_t ++;
            if(count_t > 10)
            {
                sysdPrintError("actionPlanning: Fail to sync vehicleCanInfo!\n");
                return false;
            }
        }
        else
        {
            count_t = 0;
            inputSignal.vehicleCanInfo = vehicleCanInfo_tmp;
        }
        if(nullptr == inputSignal.vehicleCanInfo)
        {
            sysdPrintError("actionPlanning: Fail to sync vehicleCanInfo!\n");
            return false;
        }

        const EgoVehicleCanStatusMsg *vehicleCanStatus_tmp = nullptr;
        vehicleCanStatus_tmp = inputPin.vehicleCanStatus_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == vehicleCanStatus_tmp)
        {
            count_t_canStatus ++;
            if(count_t_canStatus > 10)
            {
                sysdPrintError("actionPlanning: Fail to sync vehicleCanStatus!\n");
                return false;
            }
        }
        else
        {
            count_t_canStatus = 0;
            inputSignal.vehicleCanStatus = vehicleCanStatus_tmp;
        }
        if(nullptr == inputSignal.vehicleCanStatus)
        {
            sysdPrintError("actionPlanning: Fail to sync vehicleCanStatus!\n");
            return false;
        }

        inputSignal.vehicleInfo = inputPin.vehicleInfo_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == inputSignal.vehicleInfo)
        {
            sysdPrintError("actionPlanning: Fail to sync vehicleInfo!\n");
            return false;
        }

        inputSignal.actionPlannerInfo = inputPin.actionPlannerInfo_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == inputSignal.actionPlannerInfo)
        {
            sysdPrintError("actionPlanning: Fail to sync actionPlannerInfo!\n");
            return false;
        }

        inputSignal.localization = inputPin.localization_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == inputSignal.localization)
        {
            sysdPrintError("actionPlanning: Fail to sync localization!\n");
            return false;
        }

        inputSignal.objFusionData = inputPin.objFusionData_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == inputSignal.objFusionData)
        {
            sysdPrintError("actionPlanning: Fail to sync FusionData!\n");
            return false;
        }

        inputSignal.remoteVehicleInfo = inputPin.remoteVehicleInfo_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == inputSignal.remoteVehicleInfo)
        {
            sysdPrintError("actionPlanning: Fail to sync remoteVihicleInfo!\n");
            return false;
        }

        inputSignal.longitudinalControlMsg = inputPin.longitudinalControlMsg_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == inputSignal.longitudinalControlMsg)
        {
            sysdPrintError("actionPlanning: Fail to sync longitudinalControlMsg!\n");
            return false;
        }

        inputSignal.lateralControlMsg = inputPin.lateralControlMsg_in->getSyncedMsg(syncTime, syncTimeGap);
        if(nullptr == inputSignal.lateralControlMsg)
        {
            sysdPrintError("actionPlanning: Fail to sync lateralControlMsg!\n");
            return false;
        }

        return true;
    }

    inline bool interfaceRx(
        APInputPin &inputPin,
        APInputSignal &inputSignals,
        float timeout,
        float syncTimeGap)
    {
        sysd::Time oldestTimeStamp = sysd::now();
        if(false == getNewestSignals(inputPin, inputSignals, sysd::Time(timeout), &oldestTimeStamp))
        {
            sysdPrintError("actionPlanning: Fail to get newest input signals!\n");
            return false;
        }

        if(false == syncInputSignals(inputPin, inputSignals, oldestTimeStamp, sysd::Time(syncTimeGap)))
        {
            sysdPrintError("actionPlanning: Fail to synchronize the input signals!\n");
            return false;
        }

        return true;
    }

    inline bool interfaceTx(
        APOutputPin outputPin,
        DiscretizedTrajectory *discretizedTrajectory,
        EgoVehicleInfoMsg *egoVehicleInfoMsg)
    {
        outputPin.discretizedTrajectory_out->send(*discretizedTrajectory);
        outputPin.vehicleInfo_out->send(egoVehicleInfoMsg);

        return true;
    }
}   //namespace actionPlanning

#endif  //AP_INTERPROCESS_hpp