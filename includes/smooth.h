#ifndef HYBRID_ASTAR_SMOOTH_H
#define HYBRID_ASTAR_SMOOTH_H

#include <cmath>
#include <vector>

#include "node.h"
#include "dynamicVoronoi.h"
#include "vec2i.h"

namespace HybridAStar
{
    static inline float clamp(float n, float lower, float upper) {
        return std::max(lower, std::min(n, upper));
    }
    
    //判断是否为交点（内联）
    inline bool isCusp(std::vector<Node3D> path, int i)
    {
    if(i == 0 || i == (path.size() - 1)) return true;
    Vector2D xim1(path[i - 1].getX(), path[i - 1].getY());
    Vector2D xi(path[i].getX(), path[i].getY());
    Vector2D xip1(path[i + 1].getX(), path[i + 1].getY());
    Vector2D vec1 = xi - xim1;
    Vector2D vec2 = xip1 - xi;
    return vec1.dot(vec2) < 0;

    }

    //计算旋转角小函数
    float rotaAngle(Vector2D x_im1, Vector2D x_i, Vector2D x_ip1);

    //轨迹平滑器类
    class Smoother
    {
    public:
        Smoother(){};                                   //构造函数
        void smoothPath(DynamicVoronoi& voronoi);     //核心轨迹平滑函数
        void tracePath(Node3D *node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>()); //顺藤摸瓜找到轨迹点列
        std::vector<Node3D> getPath(){ return m_path; } //返回点列
        Vector2D smoothnessNewTerm(Vector2D xim1, Vector2D xi, Vector2D xip1);  //新顺滑项

        Vector2D obstacleTerm(Vector2D xi);             //障碍物项
        Vector2D curvatureTerm(Vector2D x_im2, Vector2D x_im1, Vector2D x_i, Vector2D x_ip1, Vector2D x_ip2);   //曲率项
        Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);   //顺滑项
        Vector2D voronoiTerm(Vector2D xi);              //voronoi项
        bool isOnGrid(Vector2D xi)
            { return xi.getX() >= 0 && xi.getX() < height && xi.getY() >= 0 && xi.getY() < width; } //xi点是否在图像范围
        //待平滑路径
        std::vector<Node3D> m_path;
    private:
        //碰撞图
        DynamicVoronoi voronoi;
        //惩罚边界
        float kappaMax = 1. / (param::rmin * param::RS_Scaling * 1.5);//1.1);     //最大曲率
        float obsDMax = param::obsPenaMax;              //障碍物惩罚的最大距离
        float voronoiMax = param::obsPenaMax;          //影响voronoi场的最大距离
        //权重系数
        float alpha = 0.1;                          //梯度更新衰减率
        float wObstacle = 0.002;//0.1;                         //障碍物项权重
        float wCurvature = 0.6;//0.1;                        //曲率项权重
        float wSmoothness = 0.1;                       //顺滑项权重
        float wVoronoi = 0.03;                            //voronoi项权重
        //地图尺寸
        int width;
        int height;
        
    };
}

#endif      //HYBRID_ASTAR_SMOOTH_H