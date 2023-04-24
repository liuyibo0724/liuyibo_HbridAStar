#ifndef ACTION_PLANNING_HPP
#define ACTION_PLANNING_HPP

#include <stdlib.h>
#include <vector>
#include "includes/planner.h"
#include "AP_interfaceProcess.hpp"

namespace actionPlanning
{
    //ActionPlanner类的声明和定义
    class ActionPlanner
    {
    public:
        ActionPlanner() {}                                                       //构造函数
        ~ActionPlanner() {}                                                      //析构函数
        bool init(unsigned char* data, int width, int height);                   //初始化
        bool setStartGoal(HybridAStar::Node3D start, HybridAStar::Node3D goal); 
        void updateMap(unsigned char* data, int width, int height) { m_planner->updateMap(unsigned char* data, int width, int height); }    //更新地图
        bool plan();                                                             //规划
        void whichNode(std::vector<HybridAStar::Node3D> &BSplinePathFirst);      //查找当前位置是在那个点上
        bool DDSMsgsOut();                                                       //将规划结果转换成DDS发布格式
        bool run(HybridAStar::Node3D            start, 
                 HybridAStar::Node3D            goal,
                 ActionPlannerInfo              *CurrentAPInfoPtr,
                 EgoVehicleCanInfoMsg           *InputEVCanInfoMsgPtr,
                 Localization                   *LocalizationPtr);                //运行核心函数
        void getInputMsgs(ActionPlannerInfo     *CurrentAPInfoPtr,
                          EgoVehicleCanInfoMsg  *InputEVCanInfoMsgPtr,
                          Localization          *LocalizationPtr)                //传入所需信号
        DiscretizedTrajectory* getOutputDTPtr() const { return &outputDT; }      //返回轨迹点列指针
    private: 
        HybridAStar::planner                                *m_planner;          //私有规划器
        std::pair<HybridAStar::Node3D, HybridAStar::Node3D> m_sta_goa;           //起始点和目标点
        bool                                                isRunSuccess = false;//是否规划运行成功
        int                                                 pathIdx = -1;        //当前所在路径点的序号（注意！！！到时候需要和outputDT一起发布）

        int                                                 m_width;             //图片宽度
        int                                                 m_height;            //图片高度

        ActionPlannerInfo                                   currentAPInfo;       //当前运动规划器状态信息
        EgoVehicleCanInfoMsg                                inputEVCanInfoMsg;   //输入的自车Can信息
        Localization                                        localization;        //输入的定位信息
 
        DiscretizedTrajectory                               outputDT;            //输出轨迹点列
    };
}

#endif  //ACTION_PLANNING_HPP