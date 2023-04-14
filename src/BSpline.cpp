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

void BSpline_referenceLine::fit()
{
    int order = param::BSpline_order;
    if(m_rawPath.empty()) { std::cout << "m_rawPath为空" << std::endl; return; }
    else if(order < 1) { std::cout << "拟合阶数不合理" << std::endl; return; }
    else
    {
        auto front = *(m_rawPath.begin()), back = *(-- m_rawPath.end());    //拉出待拟合Node3D点列的头尾元素
        m_rawPath.insert(m_rawPath.begin(), front);
        // m_rawPath.push_back(back);

        int pathLength = m_rawPath.size();
        for(int num = 0; num < pathLength * 5; num ++)
        {
            float position = (float)num / 5.;   //精确位置
            int i = (int)(std::floor(position));//落在[i, i + 1]区间上
            int upper_halfOrder = order / 2 + order % 2;   //半阶次长度

            HybridAStar::Vector2D vector2D_tmp;
            // vector2D_tmp = BSpline_base(order, i, position + 0.5 * (1. + (float)order)) * HybridAStar::Vector2D(m_rawPath[i].getX(), m_rawPath[i].getY());
            int j = -upper_halfOrder;
            while(j <= upper_halfOrder)
            {
                int i_j = std::min(std::max(0, i + j), pathLength - 1);     //防止i + j超出范围
                vector2D_tmp = vector2D_tmp + BSpline_base(order, i + j, position + 0.5 * (1. + (float)order))
                                 * HybridAStar::Vector2D(m_rawPath[i_j].getX(), m_rawPath[i_j].getY());
                ++ j;
            }
            HybridAStar::Node3D node3D_tmp(vector2D_tmp.getX(), vector2D_tmp.getY(), 0, 0, 0, nullptr); //生成对应的拟合后节点
            m_BSplinePath.push_back(node3D_tmp);    //拟合后节点尾插进m_BSplinePath
        }
    }
}