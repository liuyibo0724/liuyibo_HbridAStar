#ifndef HYBRID_ASTAR_SMOOTH_H
#define HYBRID_ASTAR_SMOOTH_H

#include <cmath>
#include <vector>
#include <iostream>
#include "node.h"

namespace HybridAStar
{
    //2D向量类
    class Vector2D
    {
    public:
        inline Vector2D(const double x = 0, const double y = 0){ this->x = x; this->y = y; }//构造函数
        // 计算符重载
        inline Vector2D operator + (const Vector2D &cpr) const { return Vector2D(x + cpr.x, y + cpr.y); }
        inline Vector2D operator - (const Vector2D &cpr) const { return Vector2D(x - cpr.x, y - cpr.y); }
        inline Vector2D operator * (const double &k) const { return Vector2D(x * k, y * k); }
        inline Vector2D operator / (const double &k) const { return Vector2D(x / k, y / k); }
        inline Vector2D operator - () const { return Vector2D(-x, -y); }
        friend std::ostream& operator << (std::ostream &os, const Vector2D &cpr) { os << "(" << cpr.x << ", " << cpr.y << ")"; return os; }
        //向量长度
        double length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }
        //向量长度平方
        double sqrlength() const { return x * x + y * y; }
        //向量点乘
        double dot(const Vector2D &cpr) { return x * cpr.x + y * cpr.y; }
        //求与某向量正交的分量
        inline Vector2D ort(Vector2D &cpr)
        {
            Vector2D tmpSelf(this->x, this->y);
            Vector2D result = tmpSelf - cpr * tmpSelf.dot(cpr) / cpr.sqrlength();
            return result;
        }
        // 查询类函数
        inline double getX(){ return x; }
        inline double getY(){ return y; }

    private:
        double x;
        double y;
    };
    inline Vector2D operator * (double k, const Vector2D &cpr){ return (k * cpr); }

    //轨迹平滑器类
    class Smoother
    {
    public:
        Smoother(){};                                   //构造函数
        void smoothPath(std::vector<Node3D> &path);     //核心轨迹平滑函数
        void tracePath(Node3D *node, int i = 0, std::vector<Node3D> &path = std::vector<Node3D>()); //顺藤摸瓜找到轨迹点列
        std::vector<Node3D> getPath(){ return m_path; } //返回点列
        Vector2D obstacleTerm(Vector2D xi);             //障碍物项
        Vector2D curvatureTerm(Vector2D xi0, Vector2D xi1, Vector2D xi2);   //曲率项
        Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);   //顺滑项
        // Vector2D voronoiTerm(Vector2D xi);              //voronoi项
        bool isOnGrid(Vector2D xi)
            return xi.x >= 0 && xi.x < height && xi.y >= 0 && xi.y < width; //xi点是否在图像范围
    private:
        //惩罚边界
        double kappaMax = 1. / (param::rmin * 1.1);     //最大曲率
        double obsMax = param::obsPenaMax;              //障碍物惩罚的最大距离
        double voronoiMax = param::obsPenaMax;          //影响voronoi场的最大距离
        //权重系数
        double fallRate = 0.1;                          //梯度更新衰减率
        double wObstacle = 0.1;                         //障碍物项权重
        double wCurvature = 0.1;                        //曲率项权重
        double wSmoothness = 0.5;                       //顺滑项权重
        double wVoronoi = 0;                            //voronoi项权重
        //地图尺寸
        int width;
        int height;
        //待平滑路径
        std::vector<Node3D> m_path;
    };
}

#endif      //HYBRID_ASTAR_SMOOTH_H