#ifndef AP_COMPONENT_HPP
#define AP_COMPONENT_HPP

#include "AP_interfaceProcess.hpp"
#include "Action_Planning.hpp"

using sysd::Time;

namespace actionPlanning
{

    class AP_Component : public sysd::PeriodicComponent
    {
    public:
        AP_Component() : sysd::PeriodicComponent("actionPlanning")
        {
            m_inputPin.vehicleCanInfo_in = this->createInputPinCache("EgoVehicleCanInfoMsg", EgoVehicleCanInfoMsg, 100);
            m_inputPin.vehicleCanStatus_in = this->createInputPinCache("EgoVehicleCanStatusMsg", EgoVehicleCanStatusMsg, 100);
            m_inputPin.vehicleInfo_in = this->createInputPinCache("EgoVehicleInfoMsgs", EgoVehicleInfoMsgs, 50);
            m_inputPin.actionPlannerInfo_in = this->createInputPinCache("ActionPlannerInfo", ActionPlannerInfo, 50);
            m_inputPin.localization_in = this->createInputPinCache("Localization", Localization, 100);
            m_inputPin.objFusionData_in = this->createInputPinCache("FusionData", FusionData, 100);
            m_inputPin.remoteVehicleInfo_in = this->createInputPinCache("RemoteVehicleInfoMsg", RemoteVehicleInfoMsg, 100);
            m_inputPin.longitudinalControlMsg_in = this->createInputPinCache("LongitudinalControlMsg",LongitudinalControlMsg, 100);
            m_inputPin.lateralControlMsg_in = this->createInputPinCache("LateralControlMsg", LateralControlMsg, 100);

            m_outputPin.discretizedTrajectory_out = this->createOutputPin("DiscretizedTrajectory", DiscretizedTrajectory);
            m_outputPin.vehicleInfo_out = this->createOutputPin("EgoVehicleInfoMsg", EgoVehicleInfoMsg);

            m_isAPInputSIgnalTimeOut = false;
            m_isGetNewestSignalSuccessAtStart = false;
        }
        ~AP_Component()
        {
            delete m_inputPin.vehicleCanInfo_in;
            delete m_inputPin.vehicleCanStatus_in;
            delete m_inputPin.vehicleInfo_in;
            delete m_inputPin.actionPlannerInfo_in;
            delete m_inputPin.localization_in;
            delete m_inputPin.objFusionData_in;
            delete m_inputPin.remoteVehicleInfo_in;
            delete m_inputPin.longitudinalControlMsg_in;
            delete m_inputPin.lateralControlMsg_in;

            delete m_outputPin.discretizedTrajectory_out;
            delete m_outputPin.vehicleInfo_out;
        }


        //初始化
        bool init() override    
        {

        }
        //运行
        bool run(uint32_t seq) override
        {
            
        }
    }

    private:
        ActionPlanner m_actionPlanner;
        APInputPin m_inputPin;
        APInputSignal m_inputSignal;
        APOutputPin m_outputPin;

        bool m_isAPInputSIgnalTimeOut;  //接收输入信号是否超时
        bool m_isGetNewestSignalSuccessAtStart; //开始时是否得到最新信号


}

#endif      //AP_COMPONENT_HPP