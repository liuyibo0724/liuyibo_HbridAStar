#include <corecrt_math_defines.h>
#include <iostream>
#include <boost/heap/binomial_heap.hpp>
#include <set>
#include "hybridAStar.h"

using namespace HybridAStar;
#define PI_SQR M_PI * M_PI

//Node2D点升序排列
class CompareNode2D
{
public:
    bool operator()(const Node2D &node1, const Node2D &node2) const
    {
        return node1.getC() < node2.getC();
    }
};

//Node3D点升序排列
class CompareNode3D
{
public:
    bool operator()(const Node3D &node1, const Node3D &node2) const
    {
        return node1.getC() < node2.getC();
    }
};

//构造函数
hybridAStar::hybridAStar(CollisionDetetion *map) : m_map(map)
{
    m_nodes2D = new Node2D[map->getSize()];
    m_nodes3D = new Node3D[map->getSize() * param::headings];
}

//析构函数
hybridAStar::~hybridAStar()
{
    delete [] m_nodes2D;
    delete [] m_nodes3D;
}

//传统aStar规划器返回2D末点的已付出代价G
//（注意scale的意义不明也许是缩放尺度记得修改）
double HybridAstar::hybridAStar::aStar(Node2D &start, Node2D &goal, double scale = 1)
{
    int width = m_map->getWidth();
    int height = m_map->getHeight();
    goal.setIdx(width); //设置goal的Idx
    if(!m_map->isNodeTraversable(&goal)) return 1000;   //目标不可通行直接不规划并返回1000
    int iPred, iSucc;   //当前点和子节点的Idx
    Node2D nPred, nSucc;  //当前点和子节点对象
    Node2D *node2D_tmp = nullptr;
    double newG;    //新_已付出代价
    for(int i = 0; i < m_map->getSize(); i ++)  m_nodes2D[i].reset();   //open和close集全部置空
    std::set<HybridAStar::Node2D, CompareNode2D> open;  //定义open集
    start.updateH(goal);    //start到goal的欧式距离
    start.open();   //start加入open集
    open.insert(start);    //open集插入start点
    iPred = start.setIdx(width);    //拿到goal的Idx
    m_nodes2D[iPred] = start;   //start置入m_nodes[]对应的位置
    
    //第一层循环
    while(!open.empty())
    {
        nPred = *(open.begin());    //拿到open集中代价最低的点
        iPred = nPred.setIdx(width);//拿到Idx
        //检查：如果已扩展，则从open set中移除，处理下一个
        if(m_nodes2D[iPred].isClosed()){ open.erase(nPred); continue; }
        else if(m_nodes2D[iPred].isOpen())
        {
            m_nodes2D[iPred].close();   //标记为close
            m_nodes2D[iPred].discover();//标记为已发现
            open.erase(nPred);          //从open集中移除
            //判断是否邻近目标点，若然，结束循环返回G值
            if(nPred.equal(goal, scale)){ goal.setPred(&Pred); return nPred.getG(); }
            //并未足够靠近目标点，8向寻找
            for(int i = 0; i < Node2D::dir; i ++)
            {
                node2D_tmp = nPred.createSuccessor(i, 1 / scale);//i向步进1/scale搜索子节点
                nSucc = *node2D_tmp;
                iSucc = nSucc.setIdx(width);                //拿到子节点Idx
                //网格范围检测、节点通行性检测、是否标记为close
                if(nSucc.isOnGrid(width, height)
                    && m_map->isNodeTraversable(&nSucc)
                    && !m_nodes2D[iSucc].isClosed())
                {
                    nSucc.updateG();    //更新已付出代价G
                    newG = nSucc.getG();
                    //若新路线G代价小于原代价，或子节点未在open集中
                    if(newG < m_nodes2D[iSucc].getG() || !m_nodes2D[iSucc].isOpen())
                    {
                        nSucc.updateH(goal);    //计算启发代价H
                        nSucc.open();           //nSucc置入open集
                        m_nodes2D[iSucc] = nSucc; //取代或更新原m_nodes2D[iSucc]
                        m_nodes2D[iSucc].setPred(&nPred);   //设置父节点指针为&nPred
                        open.insert(m_nodes2D[iSucc]);      //置入open集
                        delete node2D_tmp;
                    }
                    else delete node2D_tmp;
                }
                else delete node2D_tmp;
            }
        }
    }
    return 1000;
}