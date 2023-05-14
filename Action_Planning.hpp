#ifndef ACTION_PLANNING_HPP
#define ACTION_PLANNING_HPP

#include <stdlib.h>
#include <vector>
#include "includes/planner.h"
#include "includes/filter.h"
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
        void updateMap(unsigned char* data, int width, int height);              //更新地图
        bool plan();                                                             //规划
        void whichNode(std::vector<HybridAStar::Node3D> &BSplinePathFirst);      //查找当前位置是在那个点上
        bool DDSMsgsOut();                                                       //将规划结果转换成DDS发布格式
        bool run(HybridAStar::Node3D            &start, 
                 HybridAStar::Node3D            &goal,
                 ActionPlannerInfo              *CurrentAPInfoPtr,
                 EgoVehicleCanInfoMsg           *InputEVCanInfoMsgPtr,
                 Localization                   *LocalizationPtr,
                 LocalizationPtr                *StartPtr,
                 LocalizationPtr                *GoalPtr);                       //运行核心函数
        void getInputMsgs(ActionPlannerInfo     *CurrentAPInfoPtr,
                          EgoVehicleCanInfoMsg  *InputEVCanInfoMsgPtr,
                          Localization          *LocalizationPtr,
                          LocalizationPtr       *StartPtr,
                          LocalizationPtr       *GoalPtr)                        //传入所需信号
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
        Localization                                        start;               //输入的起始点信息
        Localization                                        goal;                //输入的终点信息

        //四个过滤器
        HybridAStar::signalsFilter<float> m_vechileSpeed_filter;
        HybridAStar::signalsFilter<float> m_longitude_filter;
        HybridAStar::signalsFilter<float> m_latitude_filter;
        HybridAStar::signalsFilter<float> m_headingAngle_filter;
 
        DiscretizedTrajectory                               outputDT;            //输出轨迹点列
    };

    /* 简易经纬度转换为原点为地心的空间坐标系
    先验知识：地球长轴 a = 6378137 m
                短轴 b = 6356752.3142451793 m
                椭圆偏心率 e^2 = (a^2 + b^2) / a^2 */
    //定位坐标系（经纬度）转换为地心空间坐标系
    bool LL2XYZ(double &coordi[3], double longitude, double latitude)
    {
        if(abs(longitude) > M_PI || abs(latitude) > 0.5f * M_PI) return false;
        else
        {
            double a = 6378137., b = 6356752.3142451793;
            double e_square = (a * a + b * b) / (a * a);
            double z = a * (1. - e_square) * sin(latitude) / sqrt(1. - e_square * sin(latitude) * sin(latitude));
            double r = a * cos(latitude) / sqrt(1. - e_square * sin(latitude) * sin(latitude));
            double x = r * cos(longitude), y = r * sin(longitude);
            coordi = {x, y, z};
            return true;
        }
    }
    //自定义符号函数
    inline double sgn(double val)
    { return val > 0. ? 1. : -1.; }
    //定位坐标系（经纬度）转换为基于goal点为原点的栅格地图坐标系
    bool LL2GoalGridBased(Localization goal, Localization &localization)
    {
        double goal_theta = goal.longitude, goal_alpha = goal.latitude, goal_headingAngle = goal.headingAngle;
        double localization_theta = localization.longitude, 
               localization_alpha = localization.latitude, 
               localization_headingAngle = localization.headingAngle;
        double mediation_theta = localization.longitude, 
               mediation_alpha = goal.latitude;
        double goal_XYZ[3], localization_XYZ[3], mediation_XYZ[3];
        if(LL2XYZ(goal_XYZ, goal_theta, goal_alpha) && 
           LL2XYZ(mediation_XYZ, mediation_theta, mediation_alpha) && 
           LL2XYZ(localization_XYZ, localization_theta, localization_alpha))
        {
            double x_tmp = sqrt(pow(goal_XYZ[0] - mediation_XYZ[0], 2.) + 
                                pow(goal_XYZ[1] - mediation_XYZ[1], 2.) +
                                pow(goal_XYZ[2] - mediation_XYZ[2], 2.)) * sgn(mediation_theta - goal_theta);
            double y_tmp = sqrt(pow(localization_XYZ[0] - mediation_XYZ[0], 2.) + 
                                pow(localization_XYZ[1] - mediation_XYZ[1], 2.) +
                                pow(localization_XYZ[2] - mediation_XYZ[2], 2.)) * sgn(localization_alpha - mediation_alpha);
            double phi = atan2(y_tmp, x_tmp);   //phi为线段goal->start与正东方向的夹角，范围(- M_PI, M_PI]
            /* 此处将localziation结构体转换为以goal为原点
            且goal指向为x正方向的坐标，
            并保证航向角headingAngle范围在[0, 2 * M_PI)
            单位: m, rad */
            localization.longitude = y_tmp * sin(-goal_headingAngle) + x_tmp * cos(-goal_headingAngle);
            localization.latitude = y_tmp * cos(-goal_headingAngle) - x_tmp * sin(-goal_headingAngle);
            localization.headingAngle = HybridAStar::normalizeHeadingRad(localization_headingAngle - goal_headingAngle);
            /* 此处转换单位: pixel, rad */
            localization.longitude = 20. * localization.longitude + 160;
            localization.latitude = 20. * localization.latitude + 200;
            return true;
        }
        return false;
    }
}

#endif  //ACTION_PLANNING_HPP