#include "node.h"
using namespace HybridAStar;

//正则化位姿指向角度
static inline float normalizeHeadingRad(float t)
{
    if(std::abs(t) >= 2.*M_PI)
    {
        float sign_2M_MPI = t / std::abs(t) * 2. * M_PI;
        while(std::abs(t) < 2.*M_PI)
            t -= sign_2M_MPI;
    }
    return t;
}

//Node2D创建子结点
Node2D* Node2D::createSuccessor(const int i, float inv_scale)
{
    int xSucc = x + (int)((float)Node2D::dx[i] * inv_scale);
    int ySucc = y + (int)((float)Node2D::dy[i] * inv_scale);
    return new Node2D(xSucc, ySucc, g, 0, this);
}

//更新已经付出的代价g
void Node3D::updateG()
{
    //若方向向前
    if(prim < 3)
    {
        if(pred->prim != prim) //方向改变惩罚
        {
            if(pred->prim > 2) //方向发生反转
                g += dx[0] * param::penaltyTurning * param::penaltyCOD;
            else //方向未发生反转
                g += dx[0] * param::penaltyTurning;
        }
        else g += dx[0]; //方向未改变
    }
    //若方向向后
    else
    {
        if(pred->prim != prim) //方向改变惩罚
        {
            if(pred->prim < 3) //方向未发生反转
                g += dx[0] * param::penaltyTurning * param::penaltyCOD * param::penaltyReversing;
            else //方向未发生反转
                g += dx[0] * param::penaltyTurning * param::penaltyReversing;
        }
        else g += dx[0] * param::penaltyReversing;
    }
}

//重载==
bool Node3D::operator == (const Node3D &comparator) const
{
    return (int)x == (int)comparator.x && (int)y == (int)comparator.y 
            && (std::abs(t - comparator.t) < param::deltaHeadingRad
                || std::abs(t - comparator.t) > param::deltaHeadingNegRad);
}

//Node3D创建子结点
Node3D* Node3D::createSuccessor(const int i, float inv_scale)
{
    float xSucc, ySucc, tSucc;
    //若子结点向前
    if(i < 3)
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