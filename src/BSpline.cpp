#include "BSpline.h"

using namespace BSpline;

//B样条曲线任意阶基函数
float BSpline::BSpline_base(int order, int i, float position)    //i为中心点
{
    if(position >= (float)(i + order + 1) || position < (float)i) return 0;
    else
    {
        if(order == 0) return 1.;
        float coef1 = (position - (float)i) / (float)order;   //系数1
        float coef2 = ((float)i + (float)order + 1. - position) / (float)order;    //系数2
        float result = coef1 * BSpline::BSpline_base(order - 1, i, position) + coef2 * BSpline::BSpline_base(order - 1, i + 1, position);
        return result;
    }
}

//设置待拟合Node3D点列
void BSpline_referenceLine::setRawPath(std::vector<HybridAStar::Node3D> path)
{
    std::vector<HybridAStar::Node3D> nodes3D_tmp;
    for(int i = 0; i < path.size(); i ++)
    {
        if(nodes3D_tmp.empty() && i != 0) nodes3D_tmp.push_back(path[i - 1]);   //添加头节点为上一子段的尾节点
        nodes3D_tmp.push_back(path[i]);
        if(HybridAStar::isCusp(path, i) && i != 0)
        {
            m_rawPath.push_back(nodes3D_tmp);  //若nodes3D_tmp非空则尾插进m_rawPath中去
            nodes3D_tmp.clear();               //清空整个nodes3D_tmp
        }
    }
}

//避免插值出现角度突变
float BSpline::AvoidAngleMutation(std::vector<HybridAStar::Node3D> path, int ref, int pos)
{
    float ref_t = path[ref].getT(), pos_t = path[pos].getT();
    float add[2] = {2.f * M_PI, -2.f * M_PI};
    if(fabs(pos_t - ref_t) > M_PI)
    {
        for(int i = 0; i < 2; i ++)
            if(fabs((pos_t + add[i]) - ref_t) < M_PI) return pos_t + add[i];
    }
    return pos_t;
}

//拟合核心函数
void BSpline_referenceLine::fit()
{
    int order = param::BSpline_order;
    if(m_rawPath.empty()) { std::cout << "m_rawPath为空" << std::endl; return; }
    else if(order < 1) { std::cout << "拟合阶数不合理" << std::endl; return; }
    else
    {
        std::vector<HybridAStar::Node3D> nodes3D_tmp;
        for(auto ptr = m_rawPath.begin(); ptr < m_rawPath.end(); ptr ++)
        {
            auto subPath_tmp = *ptr;    //取出m_rawPath中的子段
            
            auto front = *(subPath_tmp.begin()), back = *(-- subPath_tmp.end());    //拉出待拟合Node3D点列的头尾元素
            subPath_tmp.insert(subPath_tmp.begin(), front);
            // m_rawPath.push_back(back);

            int pathLength = subPath_tmp.size();

            //防止头尾出现0至2 * M_PI的隐藏bug
            if((subPath_tmp[1].getT() == 0.f || subPath_tmp[1].getT() == 2.f * M_PI) 
                && abs(subPath_tmp[2].getT() - subPath_tmp[1].getT()) > M_PI)
            {
                auto Var_tmp = subPath_tmp[0].getT() == 0.f ? 2.f * M_PI : 0.f;
                subPath_tmp[0].setT(Var_tmp);
                subPath_tmp[1].setT(Var_tmp);
            }  
            if((subPath_tmp[pathLength - 1].getT() == 0.f || subPath_tmp[pathLength - 1].getT() == 2.f * M_PI) 
                && abs(subPath_tmp[pathLength - 1].getT() - subPath_tmp[pathLength - 2].getT()) > M_PI)
            {
                auto Var_tmp = subPath_tmp[0].getT() == 0.f ? 2.f * M_PI : 0.f;
                subPath_tmp[pathLength - 1].setT(Var_tmp);
            }

            for(int num = 0; num < pathLength * param::BSpline_Scaling; num ++)
            {
                float position = (float)num / param::BSpline_Scaling;   //精确位置
                int i = (int)(std::floor(position));//落在[i, i + 1]区间上
                int upper_halfOrder = order / 2 + order % 2;   //半阶次长度

                HybridAStar::Vector2D vector2D_tmp;            //初始化临时2D向量
                float T_tmp = 0.f;                             //初始化临时角度
                int j = -upper_halfOrder;
                while(j <= upper_halfOrder)
                {
                    int i_j = std::min(std::max(0, i + j), pathLength - 1);     //防止i + j超出范围
                    vector2D_tmp = vector2D_tmp + BSpline_base(order, i + j, position + 0.5 * (1. + (float)order))
                                     * HybridAStar::Vector2D(subPath_tmp[i_j].getX(), subPath_tmp[i_j].getY()); //拟合x, y坐标
                    T_tmp += BSpline_base(order, i + j, position + 0.5 * (1. + (float)order)) * AvoidAngleMutation(subPath_tmp, i, i_j);//subPath_tmp[i_j].getT(); //拟合T角度
                    ++ j;
                }
                HybridAStar::Node3D node3D_tmp(vector2D_tmp.getX(), vector2D_tmp.getY(), T_tmp, 0, 0, nullptr); //生成对应的拟合后节点
                nodes3D_tmp.push_back(node3D_tmp);
            }
            setBSplinePathVelocity(nodes3D_tmp);    //设置速度并尾插进m_BSplinePath中
            nodes3D_tmp.clear();                    //清空nodes3D[]
        }
    }
}

//设置拟合后曲线速度
void BSpline_referenceLine::setBSplinePathVelocity(std::vector<HybridAStar::Node3D> &nodes3D_tmp)
{
    float revIdx;                                                                               //反转指数
    if(m_BSplinePath.empty())                                                                   //若m_BSplinePath为空根据车头指向和运动指向初始化revIdx
    {
        auto pointDirect = HybridAStar::Vector2D(cos(nodes3D_tmp[0].getT()), sin(nodes3D_tmp[0].getT()));
        auto moveDirect = HybridAStar::Vector2D(nodes3D_tmp[1].getX() - nodes3D_tmp[0].getX(), 
                                                nodes3D_tmp[1].getY() - nodes3D_tmp[0].getY());
        if(pointDirect.dot(moveDirect) >= 0) revIdx = 1.f;
        else revIdx = -1.f;
    }
    else                                                                                        //否则根据与前段反转确定revIdx
    {
        if(m_BSplinePath[m_BSplinePath.size() - 2].second > 0) revIdx = -1.f;
        else revIdx = 1.f;
    }
    
    std::vector<std::pair<HybridAStar::Node3D, float>> m_BSplinePath_tmp;
    float velocityMax = param::velocityMax * 20;                                               //读取最大速度限制
    for(int i = 0; i < nodes3D_tmp.size(); i ++) m_BSplinePath_tmp.push_back(std::make_pair(nodes3D_tmp[i], revIdx * velocityMax));  //初始化m_BSplinePath_tmp
    float sumDis = 0.;                                                                         //总距离置为0
    for(int i = 0; i < nodes3D_tmp.size(); i ++)
    {
        sumDis += sqrt(pow(nodes3D_tmp[i + 1].getX() - nodes3D_tmp[i].getX(), 2) + 
                        pow(nodes3D_tmp[i + 1].getY() - nodes3D_tmp[i].getY(), 2));            //计算相邻点距离
        if(sumDis > velocityMax) break;
        float ti = pow(sumDis / velocityMax, 1.f / 3.f);                                       //第i个节点的时间
        float vi = velocityMax * ti * ti;                                                      //第i节点的速度绝对值
        m_BSplinePath_tmp[i].second = vi * revIdx;                                                      //赋值第i个节点速度
    }
    sumDis = 0;                                                                                //总距离重置为0
    for(int i = nodes3D_tmp.size() - 1; i >= 0; i --)
    {
        sumDis += sqrt(pow(nodes3D_tmp[i].getX() - nodes3D_tmp[i - 1].getX(), 2) + 
                        pow(nodes3D_tmp[i].getY() - nodes3D_tmp[i - 1].getY(), 2));            //计算相邻点距离
        if(sumDis > velocityMax) break;
        float ti = pow(sumDis / velocityMax, 1.f / 3.f);                                       //第i个节点的时间
        float vi = velocityMax * ti * ti;                                                      //第i节点的速度绝对值
        if(abs(m_BSplinePath_tmp[i].second) > vi) m_BSplinePath_tmp[i].second = vi * revIdx;   //赋值第i个节点速度，并考虑反转指数revIdx
    }
    for(int i = 0; i < m_BSplinePath_tmp.size(); i ++) m_BSplinePath.push_back(m_BSplinePath_tmp[i]);
}