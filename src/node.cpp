#include "node.h"
using namespace HybridAStar;

//正则化位姿指向角度
static inline double normalizeHeadingRad(double t)
{
    if(std::abs(t) >= 2.*M_PI)
    {
        double sign_2M_MPI = t / std::abs(t) * 2. * M_PI;
        while(std::abs(t) < 2.*M_PI)
            t -= sign_2M_MPI;
    }
    return t;
}

//Node2D创建子结点
Node2D* Node2D::createSuccessor(const int i, double inv_scale)
{
    int xSucc = x + (int)((double)Node2D::dx[i] * inv_scale);
    int ySucc = y + (int)((double)Node2D::dy[i] * inv_scale);
    return new Node2D(xSucc, ySucc, g, 0, this);
}

//Node3D创建子结点
Node3D* createSuccessor(const int i, double inv_scale)
{
    double xSucc, ySUcc, tSucc;
    //若子结点向前
    if(i > 3)
    {
        xSucc = x + (dx[i] * cos(t) - dy[i] * sin(t)) * inv_scale;
        ySucc = y + (dx[i] * sin(t) + dy[i] * cos(t)) * inv_scale;
        tSucc = normalizeHeadingRad(t + dt[i]);
    }
    //若子结点向后
    else
    {
        xSucc = x - (dx[i - 3] * cos(t) + dy[i - 3] * sin(t)) * inv_scale;
        ySucc = y - (dx[i - 3] * sin(t) - dy[i - 3] * cos(t)) * inv_scale;
        tSucc = normalizeHeadingRad(t - dt[i - 3]);
    }
    return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}