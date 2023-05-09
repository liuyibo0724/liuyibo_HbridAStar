#ifndef HYBRID_ASTAR_PARAM_H
#define HYBRID_ASTAR_PARAM_H

#include <math.h>
#include <vector>
#include <set>
/*定义hybrid_astar算法基础参数*/
namespace param
{
    //hybrid_astar的planner的最大搜索迭代数[#]
    static const int iterations = 50000;
    //开始RS搜索阈值比例
    static const float RS_SearchPoint = 1.0;//0.5;
    //自车padding宽度[m]
    static const float padding = 1.5;  //0.15;
    //自车宽度（KIWI_EV)[m]
    static const double width =  33;   //1.655 + 2.*padding;
    //自车长度（KIWI_EV）[m]
    static const double length =  58;  //2.894 + 2.*padding;    //  像素和距离换算比例为20
    //阿克曼中心距离前端比例*2
    static const float front2Rate = 1.6;
    //阿克曼中心距离后端比例*2
    static const float rear2Rate = 0.4;
    //自车前后轴距离
    static const float alxesDis = 40.4;
    //最小转弯半径（需适配，先按照RS算法方便设为1）[m]
    static const float rmin = 1.;
    //车位宽度
    static const float parkingSpaceWidth = 3 * 20;
    //车位长度
    static const float parkingSpaceLength = 3 * 20;
    //阿克曼中心距离车位前端比例*2
    static const float parkingFront2Rate = 1.;
    //阿克曼中心距离车位后端比例*2
    static const float parkingRear2Rate = 1.;
    //RS曲线半径缩放因子
    static const float RS_Scaling = 80.f;//40.;
    //sta2pred插值缩放因子
    static const int sta2predShoot_Scaling = 10;
    //航向角离散数[#]
    static const int headings = 120;
    //航向角离散步长（度）[°]
    static const float deltaHeadingDeg = 360 / (double) headings;
    //航向角离散步长（弧度）[rad]
    static const float deltaHeadingRad = 2. * M_PI / (double) headings;
    //航向角离散步长负向（弧度）[rad]
    static const float deltaHeadingNegRad = 2. * M_PI - deltaHeadingRad;
    //2D网格尺寸[m]
    static const float cellSize = 1.;
    //代价计算误差小余量[m]
    static const float tieBreaker = 0.01;
    //cell中坐标离散分辨率[#]
    static const int positionResolution = 10;
    //cell中离散位置数量[#]
    static const int positions = positionResolution*positionResolution;
    //相对cell中心的坐标[m]
    struct relPos
    {
        int x;
        int y;
        //判断相等
        bool operator == (const relPos &comparator) const 
            { return x == comparator.x && y == comparator.y; }
        //判断大小
        bool operator  < (const relPos &comparator) const
        {
            if(x != comparator.x) return x < comparator.x;
            else return y < comparator.y;
        }
    };
    //位姿所占用的cells
    //struct poseConfig
    //{
    //    int length;
    //    relPos pos[64];
    //};
    typedef std::set<relPos> poseConfig;
    //转向惩罚[#]
    static const float penaltyTurning = 4.0;//1.05;
    //倒车惩罚（运动图元>2）[#]
    static const float penaltyReversing = 5.;  //25.0;
    //改变方向惩罚（运动图元由<3到>2）[#]
    static const float penaltyCOD = 10.;  //50.0;
    static const float dubinsShotDistance = 100.0;
    //障碍物最大惩罚距离
    static const float obsPenaMax = 60.0;
    //最大允许速度（5 km/h）
    static const float velocityMax = 1.388;
    //光顺最大迭代次数
    static const int smoothMaxIterations = 500;
    //BSpline拟合阶次
    static const int BSpline_order = 3;

    /*子节点搜索步长相关参数*/
    static const float dt = 0.441786375;
    static const float dx = 1.7671455;
    static const float dx_sin = 1.71022;
    static const float dy = 0.38404267;
}

#endif //HYBRID_ASTAR_PARAM_H