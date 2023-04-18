#include "Action_Planning.hpp"

using namespace actionPlanning;

ActionPlanner::ActionPlanner() {}

ActionPlanner::~ActionPlanner() { delete m_planner; } //析构函数

bool ActionPlanner::init(unsigned char* data, int width, int height)
{
    m_planner = new HybridAStar::planner(data, width, height);
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
    if(pathIdx == -1)
    {
        if(abs(x - BSplinePathFirst[0].getX()) < param::tieBreaker &&
           abs(y - BSplinePathFirst[0].getY()) < param::tieBreaker &&
           (abs(t - BSplinePathFirst[0].getT()) < param::deltaHeadingRad || 
            abs(t - BSplinePathFirst[0].getT()) > param::deltaHeadingNegRad))
            pathIdx = 0;
        else pathIdx = -1;
    }
    else
    {
        float disSquaSelf = pow(x - BSplinePathFirst[pathIdx].getX(), 2.) +
                            pow(y - BSplinePathFirst[pathIdx].gety(), 2.) +
                            pow(t - BSplinePathFirst[pathIdx].getT(), 2.);
        float disSquaSucc;
        if(pathIdx < BSplinePathFirst.size() - 1)
            disSquaSucc = pow(x - BSplinePathFirst[pathIdx + 1].getX(), 2.) +
                          pow(y - BSplinePathFirst[pathIdx + 1].gety(), 2.) +
                          pow(t - BSplinePathFirst[pathIdx + 1].getT(), 2.);
        else disSquaSucc = 1.f;

        if(disSquaSelf > 1.f || disSquaSucc > 1.f) pathIdx = -1;
        else if(disSquaSelf <= disSquaSucc) pathIdx = pathIdx;
        else pathIdx += 1;
    }
}

bool ActionPlanner::DDSMsgsOut()
{
    if(!isRunSuccess) return false;
    auto firstBSplinePath = *(m_planner->getBSplinePath().begin());
    whichNode(firstBSplinePath.first);  //找到哪个点匹配当前位置
    outputDT.seq = pathIdx;             //赋值序列seq
    if(pathIdx == -1)
    {
        isRunSuccess = false;
        m_sta_goa.first = HybridAStar::Node3D(localization.longitude, localization.latitude, localization.headingAngle, 0, 0, nullptr,);    //更新起始点
        plan();     //重新规划
        firstBSplinePath = *(m_planner->getBSplinePath().begin());
        outputDT.isPlanSucceeded = isRunSuccess;
        for(int i = 0; i < firstBSplinePath.size(); i ++)
        {
            outputDT.discretizedTrajectory[i].pathPoint.x = firstBSplinePath[i].first.getX();
            outputDT.discretizedTrajectory[i].pathPoint.y = firstBSplinePath[i].first.getY();
            outputDT.discretizedTrajectory[i].pathPoint.kappa = firstBSplinePath[i].first.getT();
            outputDT.discretizedTrajectory[i].Vel = firstBSplinePath[i].first.getX();
        }
        outputDT.stamp = sysd::now();
        whichNode(firstBSplinePath.first);  //找到哪个点匹配当前位置
        outputDT.seq = pathIdx;             //赋值序列seq
    }
    return true;
}

void ActionPlanner::getInputMsgs(ActionPlannerInfo     &CurrentAPInfo,
                                 EgoVehicleCanInfoMsg  &InputEVCanInfoMsg,
                                 Localization          &Localization)
{
    currentAPInfo = CurrentAPInfo;
    inputEVCanInfoMsg = InputEVCanInfoMsg;
    localization = Localization;
}

void ActionPlanner::getOutputDT(DiscretizedTrajectory &OutputDT)
{ OutputDT = OutputDT; }