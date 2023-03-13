#ifndef HYBRID_ASTAR_PARAM_H
#define HYBRID_ASTAR_PARAM_H

#include <math.h>
/*定义hybrid_astar算法基础参数*/
namespace param
{
    //hybrid_astar的planner的最大搜索迭代数[#]
    static const int iterations = 50000;
    //自车padding宽度[m]
    static const double padding = 0.15;
    //自车宽度（KIWI_EV)[m]
    static const double width = 1.655 + 2.*padding;
    //自车长度（KIWI_EV）[m]
    static const double length = 2.894 + 2.*padding;
    //最小转弯半径（需适配，先按照RS算法方便设为1）[m]
    static const double rmin = 1.;
    //航向角离散数[#]
    static const int headings = 120;
    //航向角离散步长（度）[°]
    static const double deltaHeadingDeg = 360 / (double) headings;
    //航向角离散步长（弧度）[rad]
    static const double deltaHeadingRad = 2.*M_PI / (double) headings;
    //航向角离散步长负向（弧度）[rad]
    static const double deltaHeadingNegRad = 2*M_PI - deltaHeadingRad;
    //2D网格尺寸[m]
    static const double cellSize = 1.;
    //代价计算误差小余量[m]
    static const double tieBreaker = 0.01;
    //cell中坐标离散分辨率[#]
    static const int positionResolution = 10;
    //cell中离散位置数量[#]
    static const int positionResolution*positionResolution;
    //相对cell中心的坐标[m]
    struct relPos
    {
        int x;
        int y;
    };
    //亚格子查询配置
    struct subCellConfig
    {
        int length;
        relPos pos[64];
    };
}

#endif //HYBRID_ASTAR_PARAM_H