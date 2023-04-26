#ifndef PLANNER_H
#define PLANNER_H

#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include "node.h"
#include "hybridAStar.h"
#include "smooth.h"
#include "dynamicVoronoi.h"
#include "BSpline.h"

namespace HybridAStar
{
    class planner
    {
    public:
        planner(unsigned char* data, int width, int height);
        ~planner();
        void setStart(Node3D start) { m_start = start; isStartSet = true; } //设置m_start
        void setGoal(Node3D goal) { m_goal = goal; isGoalSet = true; }      //设置m_goal
        void updateMap(unsigned char* data, int width, int height);   //更新地图
        void plan();  //规划核心函数
        std::vector<std::vector<std::pair<HybridAStar::Node3D, float>>> getBSplinePath() const { return m_BSplinePaths; }    //获取最终经过平滑和B样条曲线插值的参考轨迹
    private:
        Node3D m_start;     //搜索起始点
        Node3D m_goal;      //搜索目标点
        bool isStartSet = false;    //m_start是否设置
        bool isGoalSet = false;     //m_goal是否设置

        CollisionDetection *m_map_data;
        hybridAStar *m_planner;
        DynamicVoronoi *m_voronoi;
        Smoother *m_smoother;
        BSpline::BSpline_referenceLine *m_referenceLine;

        std::vector<std::vector<std::pair<HybridAStar::Node3D, float>>> m_BSplinePaths;    //最终经过平滑和B样条曲线插值的参考轨迹
    };
}

#endif  //PLANNER_H