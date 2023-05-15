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
            m_inputPin.start_in = this->createInputPinCache("Start", Localization, 100);
            m_inputPin.goal_in = this->createInputPinCache("Goal", Localization, 100);
            // m_inputPin.map_in = this->createInputPinCache("Map", Mapdat, 100);  //Mapdata由于上游适配问题先注释隐去
            m_inputPin.objFusionData_in = this->createInputPinCache("FusionData", FusionData, 100);
            m_inputPin.remoteVehicleInfo_in = this->createInputPinCache("RemoteVehicleInfoMsg", RemoteVehicleInfoMsg, 100);
            m_inputPin.longitudinalControlMsg_in = this->createInputPinCache("LongitudinalControlMsg",LongitudinalControlMsg, 100);
            m_inputPin.lateralControlMsg_in = this->createInputPinCache("LateralControlMsg", LateralControlMsg, 100);

            m_outputPin.discretizedTrajectory_out = this->createOutputPin("DiscretizedTrajectory", DiscretizedTrajectory);
            // m_outputPin.vehicleInfo_out = this->createOutputPin("EgoVehicleInfoMsg", EgoVehicleInfoMsg);

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
            delete m_inputPin.start_in;
            delete m_inputPin.goal_in;
            delete m_inputPin.goal_in;
            delete m_inputPin.objFusionData_in;
            delete m_inputPin.remoteVehicleInfo_in;
            delete m_inputPin.longitudinalControlMsg_in;
            delete m_inputPin.lateralControlMsg_in;

            delete m_outputPin.discretizedTrajectory_out;
            // delete m_outputPin.vehicleInfo_out;

            /* (临时的)释放空地图内存 */
            delete [] m_tmp_openSpaceMap.data;
        }


        //初始化
        bool init() override    
        {
            bool rxSuccess = interfaceRx(m_inputPin, m_inputSignal, 0.1, 0.08); //最后的timeout和syncTimeGap时间仍需斟酌！
            // m_actionPlanner.init(m_inputSignal.map.data, m_inputSignal.map.width, m_inputSignal.map.height);    //初始化
            setOpenSpaceMap();                                                                                     //(临时的)设置空地图
            m_actionPlanner.init(m_tmp_openSpaceMap.data, m_tmp_openSpaceMap.width, m_tmp_openSpaceMap.height);    //(临时的)暂时使用空地图
            //给四个滤波器一口气填满m_signals[]
            m_actionPlanner.m_m_vechileSpeed_filter.setAllSignals(m_inputSignal.vehicleCanInfo.vehicleSpeed);
            m_actionPlanner.m_longitude_filter.setAllSignals(m_inputSignal.localization.longitude);
            m_actionPlanner.m_latitude_filter.setAllSignals(m_inputSignal.localization.latitude);
            m_actionPlanner.m_headingAngle_filter.setAllSignals(m_inputSignal.localization.headingAngle);
        }
        //运行
        bool run(uint32_t seq) override
        {
            bool rxSuccess = interfaceRx(m_inputPin, m_inputSignal, 0.1, 0.08); //最后的timeout和syncTimeGap时间仍需斟酌！
            
            
            // if(m_inputSignal.map.isChanged)
            // {
            //     m_actionPlanner.updateMap(m_inputSignal.map.data, m_inputSignal.map.width, m_inputSignal.map.height);    //若更新标签为真则更新地图
            //     //给四个滤波器一口气填满m_signals[]
            //     m_actionPlanner.m_m_vechileSpeed_filter.setAllSignals(m_inputSignal.vehicleCanInfo.vehicleSpeed);
            //     m_actionPlanner.m_longitude_filter.setAllSignals(m_inputSignal.localization.longitude);
            //     m_actionPlanner.m_latitude_filter.setAllSignals(m_inputSignal.localization.latitude);
            //     m_actionPlanner.m_headingAngle_filter.setAllSignals(m_inputSignal.localization.headingAngle);
            // }
            
            m_actionPlanner.run(m_start, 
                                m_goal,
                                m_inputSignal.actionPlannerInfo,
                                m_inputSignal.vehicleCanInfo,
                                m_inputSignal.localization,
                                m_inputSignal.start,
                                m_inputSignal.goal);
            interfaceTx(m_outputPin, m_actionPlanner.getOutputDTPtr());
            return true;
        }

        /* (临时函数)生成空的栅格地图 */
        void setOpenSpaceMap()
        {
            unsigned char* data = new unsigned char[320 * 400];
            for(int i = 0; i < 320 * 400; i ++) data[i] = 255;
            m_tmp_openSpaceMap.data = data;
            m_tmp_openSpaceMap.width = 400;
            m_tmp_openSpaceMap.height = 320;
            m_tmp_openSpaceMap.isChanged = false;
        }
    }

    private:
        ActionPlanner m_actionPlanner;
        APInputPin m_inputPin;
        APInputSignal m_inputSignal;
        APOutputPin m_outputPin;

        HybridAStar::Node3D m_start;
        HybridAStar::Node3D m_goal;

        bool m_isAPInputSIgnalTimeOut;  //接收输入信号是否超时
        bool m_isGetNewestSignalSuccessAtStart; //开始时是否得到最新信号

        /* (临时的)空地图 */
        Mapdata m_tmp_openSpaceMap;
}

#endif      //AP_COMPONENT_HPP