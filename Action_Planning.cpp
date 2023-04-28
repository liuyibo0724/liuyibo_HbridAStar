#include "Action_Planning.hpp"

using namespace actionPlanning;

ActionPlanner::ActionPlanner() {}

ActionPlanner::~ActionPlanner() { delete m_planner; } //析构函数

bool ActionPlanner::init(unsigned char* data, int width, int height)
{
    m_planner = new HybridAStar::planner(data, width, height);
    m_width = width;
    m_height = height;
    return true;
}

bool ActionPlanner::plan()
{
    m_planner->setStart(m_sta_goa.first);
    m_planner->setGoal(m_sta_goa.second);
    m_planner->plan();
    isRunSuccess = true;
    return true;
}

bool ActionPlanner::setStartGoal(HybridAStar::Node3D start, HybridAStar::Node3D goal)
{
    m_sta_goa.first = start;
    m_sta_goa.second = goal;
    return true;
}

void ActionPlanner::whichNode(std::vector<HybridAStar::Node3D> &BSplinePathFirst)
{
    float x = localization.longitude;
    float y = localization.latitude;
    float t = localization.headingAngle;
    //只有与[0]点重合patnIdx才能从-1转为0
    if(pathIdx == -1)   
    {
        if(abs(x - BSplinePathFirst[0].getX()) < param::tieBreaker &&
           abs(y - BSplinePathFirst[0].getY()) < param::tieBreaker &&
           (abs(t - BSplinePathFirst[0].getT()) < param::deltaHeadingRad || 
            abs(t - BSplinePathFirst[0].getT()) > param::deltaHeadingNegRad))
            pathIdx = 0;
        else pathIdx = -1;
    }
    //若超出轨迹线条外pathIdx转为-1
    else if(pathIdx == BSplinePathFirst.size() - 1)
    {
        if(abs(x - BSplinePathFirst[pathIdx].getX()) > param::tieBreaker ||
           abs(y - BSplinePathFirst[pathIdx].getY()) > param::tieBreaker ||
           (abs(t - BSplinePathFirst[pathIdx].getT()) > param::deltaHeadingRad && 
            abs(t - BSplinePathFirst[pathIdx].getT()) < param::deltaHeadingNegRad)) pathIdx = -1;
    }
    //其余情况靠近本点pathIdx即为本点靠近下一点即为下一点，若距离两点均太远置为-1
    else
    {
        float disSquaSelf = pow(x - BSplinePathFirst[pathIdx].getX(), 2.) +
                            pow(y - BSplinePathFirst[pathIdx].gety(), 2.) +
                            pow(t - BSplinePathFirst[pathIdx].getT(), 2.);
        float disSquaSucc = pow(x - BSplinePathFirst[pathIdx + 1].getX(), 2.) +
                            pow(y - BSplinePathFirst[pathIdx + 1].gety(), 2.) +
                            pow(t - BSplinePathFirst[pathIdx + 1].getT(), 2.);

        if(disSquaSelf > 2.f && disSquaSucc > 2.f) pathIdx = -1;

        else if(abs(x - BSplinePathFirst[pathIdx + 1].getX()) < param::tieBreaker &&
           abs(y - BSplinePathFirst[pathIdx + 1].getY()) < param::tieBreaker &&
           (abs(t - BSplinePathFirst[pathIdx + 1].getT()) < param::deltaHeadingRad || 
            abs(t - BSplinePathFirst[pathIdx + 1].getT()) > param::deltaHeadingNegRad)) pathIdx ++;
    }
}

bool ActionPlanner::DDSMsgsOut()
{
    //如果Run成功了那直接查点列序号
    if(isRunSuccess)
    {
        auto firstBSplinePath = *(m_planner->getBSplinePath().begin());
        whichNode(firstBSplinePath.first);  //找到哪个点匹配当前位置
        outputDT.seq = pathIdx;             //赋值序列seq
        if(pathIdx == -1) isRunSuccess = false;
    }
    //如果没有Run过或者Run过期了那就重新Run点列
    if(!isRunSuccess)
    {
        m_sta_goa.first = HybridAStar::Node3D(localization.longitude, localization.latitude, localization.headingAngle, 0, 0, nullptr,);    //更新起始点
        plan();     //重新规划
        auto firstBSplinePath = *(m_planner->getBSplinePath().begin());
        outputDT.isPlanSucceeded = isRunSuccess;
        for(int i = 0; i < firstBSplinePath.size(); i ++)   //赋值x, y坐标和航向角kappa和速度Vel
        {
            outputDT.discretizedTrajectory[i].pathPoint.x = firstBSplinePath[i].first.getX();
            outputDT.discretizedTrajectory[i].pathPoint.y = firstBSplinePath[i].first.getY();
            outputDT.discretizedTrajectory[i].pathPoint.kappa = firstBSplinePath[i].first.getT();
            outputDT.discretizedTrajectory[i].Vel = firstBSplinePath[i].second;
        }
        //利用isCusp函数查找所有分段点
        std::vector<int> subSegs;
        for(int i = 0; i < firstBSplinePath.size(); i ++) if(HybridAStar::isCusp(firstBSplinePath, i)) subSegs.push_back(i);
        //分别处理每个子段，防止分段点的影响，每个字段左闭右开
        for(int j = 0; j < subSegs.size() - 1; j ++)
        {
            int sta = subSegs[j], end = subSegs[j + 1];
            for(int i = sta; i < end; i ++)                                     //赋值车轮转向角theta, Acc, Jerk
            {
                int lowerIdx = std::max(sta, i - 1);                            //带边界限制的下指标
                int upperIdx = std::min(end - 1, i + 1);                        //带边界限制的上指标
                //前后线段中点分别设为P0和P1
                //P0数据
                float P0_x = (outputDT.discretizedTrajectory[lowerIdx].pathPoint.x +
                              outputDT.discretizedTrajectory[i].pathPoint.x) / 2.f;
                float P0_y = (outputDT.discretizedTrajectory[lowerIdx].pathPoint.y +
                              outputDT.discretizedTrajectory[i].pathPoint.y) / 2.f;
                float P0_kappa = (outputDT.discretizedTrajectory[lowerIdx].pathPoint.kappa +
                                  outputDT.discretizedTrajectory[i].pathPoint.kappa) / 2.f;
                float P0_Vel = (outputDT.discretizedTrajectory[lowerIdx].Vel +
                                outputDT.discretizedTrajectory[i].Vel) / 2.f;
                //P1数据
                float P1_x = (outputDT.discretizedTrajectory[upperIdx].pathPoint.x +
                              outputDT.discretizedTrajectory[i].pathPoint.x) / 2.f;
                float P1_y = (outputDT.discretizedTrajectory[upperIdx].pathPoint.y +
                              outputDT.discretizedTrajectory[i].pathPoint.y) / 2.f;
                float P1_kappa = (outputDT.discretizedTrajectory[upperIdx].pathPoint.kappa +
                                  outputDT.discretizedTrajectory[i].pathPoint.kappa) / 2.f;
                float P1_Vel = (outputDT.discretizedTrajectory[upperIdx].Vel +
                                outputDT.discretizedTrajectory[i].Vel) / 2.f;

                float delta_kappa = P1_kappa - P0_kappa;
                float P1_P0_length = sqrt(pow(P1_x - P0_x, 2.f) + pow(P1_y - P0_y, 2.f));
                float delta_time = P1_P0_length / outputDT.discretizedTrajectory[i].Vel;

                float theta = atan2(delta_kappa * param::alxesDis, P1_P0_length);   //注意！！！此theta值取值范围为[-M_PI/2, M_PI/2]
                outputDT.discretizedTrajectory[i].pathPoint.theta = theta;

                float Acc = (P1_Vel - P0_Vel) / delta_time;
                outputDT.discretizedTrajectory[i].Acc = Acc;
                outputDT.discretizedTrajectory[i].Jerk = 0.;
            }
        }
        //终末点Vel, theta, Acc, Jerk全部赋值为0
        outputDT.discretizedTrajectory[firstBSplinePath.size() - 1].Vel = 0.;
        outputDT.discretizedTrajectory[firstBSplinePath.size() - 1].pathPoint.theta = 0.;
        outputDT.discretizedTrajectory[firstBSplinePath.size() - 1].Acc = 0.;
        outputDT.discretizedTrajectory[firstBSplinePath.size() - 1].Jerk = 0.;

        outputDT.stamp = sysd::now();
        whichNode(firstBSplinePath.first);  //找到哪个点匹配当前位置
        outputDT.seq = pathIdx;             //赋值序列seq
    }
    return true;
}

void ActionPlanner::getInputMsgs(ActionPlannerInfo     *CurrentAPInfoPtr,
                                 EgoVehicleCanInfoMsg  *InputEVCanInfoMsgPtr,
                                 Localization          *LocalizationPtr)
{
    currentAPInfo = *CurrentAPInfoPtr;
    inputEVCanInfoMsg = *InputEVCanInfoMsgPtr;
    localization = *LocalizationPtr;
}

// void ActionPlanner::getOutputDT(DiscretizedTrajectory &OutputDT)
// { OutputDT = OutputDT; }

bool ActionPlanner::run(HybridAStar::Node3D            start, 
                        HybridAStar::Node3D            goal,
                        ActionPlannerInfo              *CurrentAPInfoPtr,
                        EgoVehicleCanInfoMsg           *InputEVCanInfoMsgPtr,
                        Localization                   *LocalizationPtr)
{
    getInputMsgs(CurrentAPInfoPtr,
                 InputEVCanInfoMsgPtr,
                 LocalizationPtr);
    setStartGoal(start, goal);
    if(start.getX() < 0 || start.getX() > m_height
       start.getY() < 0 || start.getY() > m_width
       goal.getX() < 0 || goal.getX() > m_height
       goal.getY() < 0 || goal.getY() > m_width) return false;
    DDSMsgsOut();
    return true;
}

void ActionPlanner::updateMap(unsigned char* data, int width, int height)
{
    m_planner->updateMap(unsigned char* data, int width, int height);
    m_sta_goa = std::make_pair(HybridAStar::Node3D(), HybridAStar::Node3D());
    isRunSuccess = false;

    m_width = width;
    m_height = height;
}