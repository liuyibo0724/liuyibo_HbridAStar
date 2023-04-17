#ifndef BSPLINE_H
#define BSPLINE_H

#include <iostream>
#include <vector>
#include <cmath>
#include "param.h"
#include "dynamicVoronoi.h"
#include "node.h"
#include "smooth.h"

namespace BSpline
{
    //B样条曲线任意阶基函数
    float BSpline_base(int order, int i, float position);    //i为中心点

    //BSpline局部规划器
    class BSpline_referenceLine
    {
    public:
        std::vector<std::pair<HybridAStar::Node3D, float>> getBSplinePath() const { return m_BSplinePath; }   //访问拟合后的平滑参考线
        void setRawPath(std::vector<HybridAStar::Node3D> path);                             //设置待拟合Node3D点列
        void fit();                                                                         //拟合核心函数
        void clearRawPath() { m_rawPath.clear(); }                                          //清空待拟合Node3D点列
        void clearBSplinePath() { m_BSplinePath.clear(); }                                  //清空拟合后的平滑参考线
        void setBSplinePathVelocity(std::vector<HybridAStar::Node3D> &nodes3D_tmp);         //设置拟合后曲线速度
    private:
        std::vector<std::vector<HybridAStar::Node3D>> m_rawPath;     //导入的Node3D点组成的全局规划点列
        std::vector<std::pair<HybridAStar::Node3D, float>> m_BSplinePath; //经过BSpline拟合后的平滑参考线
    };
}

#endif  //BSPLINE_H