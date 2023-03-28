#include <math.h>
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
    start.open();   //start标记为open
    open.insert(start);    //open集插入start点
    iPred = start.setIdx(width);    //拿到start的Idx
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
            if(nPred.equal(goal, scale)){ goal.setPred(&m_nodes2D[iPred]); return m_nodes2D[iPred].getG(); }
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
                        m_nodes2D[iSucc].setPred(&m_nodes2D[iPred]);   //设置父节点指针为&nPred
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

//hybridAStar搜索路径的核心规划器，返回3D点列
//（注意scale的意义不明也许是缩放尺度记得修改）
Node3D* HybridAstar::hybridAStar::search_planner(Node3D &start, Node3D &goal, double scale = 0.5)
{
    int width = m_map->getWidth();
    int height = m_map->getHeight();
    int goal_id = goal.setIdx(m_map->getWidth(), m_map->getHeight());   //拿到goal的Idx
    if(!m_map->isNodeTraversable(&goal)){ return nullptr; }     //若goal不可通行返回空指针
    int iPred, iSucc;   //当前节点和子节点的Idx
    double newG;    //新_已付出代价
    for(int i = 0; i < m_map->getSize() * param::headings; i ++) m_nodes3D[i].reset();  //open和close集全部置空
    int iterations = 0;     //迭代计数
    m_shootSuccess = false;     //初始化RS射入goal布尔值
    std::set<HybridAStar::Node3D, CompareNode3D> open;  //定义open集
    updateH(start, goal);   //start到goal的启发代价
    start.open();   //start标记为open
    open.insert(start); //open集插入start点
    iPred = start.setIdx(width, height);    //拿到start的Idx
    m_nodes3D[iPred] = start;   //start置入m_nodes[]对应的位置
    double sta2goa_distance = (start.getX() - goal.getX())
                                * (start.getX() - goal.getX())
                                + (start.getY() - goal.getY())
                                * (start.getY() - goal.getY()); //start到goal总距离的平方
    Node3D nPred, nSucc;    //当前节点和子节点对象
    Node3D *node3D_tmp = nullptr, *nPred_tmp_ptr = nullptr;

    //第一层循环
    while(!open.empty())
    {
        nPred = *(open.begin());    //拿到open集中代价最低的点
        iPred = nPred.setIdx(width, height);//拿到Idx
        iterations ++;  //迭代计数+1
        //检查：如果已扩展，则从open set中移除，处理下一个
        if(m_nodes3D[iPred].isClosed()){ open.erase(nPred); continue; }
        else if(m_nodes3D[iPred].isOpen())
        {
            m_nodes3D[iPred].close();   //标记为close
            open.erase(nPred);          //从open集种删除
            //检查：当前节点是否是goal或是否到达最大迭代次数
            if(nPred == goal || iteration > param::iterations)
                { goal.setPred(&m_nodes3D[iPred]); return &m_nodes3D[iPred] }
            double current_distance = (nPred.getX() - goal.getX())
                                        * (nPred.getX() - goal.getX())
                                        + (nPred.getY() - goal.getY())
                                        * (nPred.getX() - goal.getY()); //nPred到goal距离平方
            //如果当前距离进入总距离的后半程，开始RS射
            if(current_distance < 0.5 * sta2goa_distance)
            {
                m_RS_path = m_RS.plan(ReedsShepp::pos(nPred.getX(), nPred.getY(), nPred.getT()),
                                      ReedsShepp::pos(goal.getX(), goal.getY(), goal.getT()));
                //如果RS射成功
                if(m_RS.isTraversable(&nPred, &m_RS_path, m_map))
                {
                    m_shootSuccess = true;  //RS射入成功
                    nPred_tmp_ptr = &nPred; //将nPred的地址暂存在nPred_tmp_ptr中
                    goal.setPred(nPred_tmp_ptr);
                    bool fix[4] = {false};  //判别组间是否变档的bool数组，变档则置为true
                    for(int i = 1; i < 5; i ++)
                        if(m_RS_path.length_[i-1] * m_RS_path.length_[i] < 0) fix[i-1] = true;
                    int seg_num = 0;    //5段中第几段路线
                    auto seg_length = (double)abs(m_RS_path.length_[0]);    //已遍历过子段长度和
                    bool last_fix = false;  //最新一个fix
                    for(int i = 1; i < m_RS_path.length() * 2 - 1; i ++)
                    {
                        int prim = -1;  //默认只和1段有关
                        double t = (double)i / (m_RS_path.length() * 2);    //t表示整个RS曲线百分比
                        ReedsShepp::pos p;
                        ReedsShepp::pos st(goal.getPred()->getX(), 
                                           goal.getPred()->getY(), 
                                           goal.getPred()->getT());     //以goal的临时父节点作为起点
                        interpolate(&st, t, &p);    //插值
                        //到达段间分界线时
                        if(t > seg_length / m_RS_path.length())
                        {
                            if(fix[seg_num])
                            {
                                prim = -2;  //若段间变档则与2段有关
                                nPred.setPrim(-2);
                                last_fix = true;
                            }
                            seg_num ++;
                            seg_length += (double)abs(m_RS_path.length_[seg_num]);
                        }
                        if(last_fix && prim != -2){ prim = -2; last_fix = false; }  //针对新段第一段
                        ReedsShepp::pos p1;     //设置p的微扰偏移点p1
                        double t1 = (double)(i + 0.1) / (m_RS_path.length() * 2);    //t1表示p1点百分比
                        interpolate(&st, t1, &p1);
                        double d_x = (-p1.x + p.x);     //p与p1的x差值
                        double angle = atan(d_x / (p1.y - p.y));    //负互补角
                        if(d_x > 0)
                        {
                            if(angle < 0) angle += M_PI;
                        }
                        else
                        {
                            if(angle > 0) angle -= M_PI;
                        }
                        double angle2 = p.angle - 0.5 * M_PI;
                        if(fabs(angle - angle2) > 0.2)
                        {
                            auto tmp_node =  new Node3D(p.x, p.y, angle - 0.5*M_PI, -1, -1, nPred_tmp_ptr, prim);
                            nPred_tmp_ptr = tmp_node;
                        }
                        else
                        {
                            auto tmp_node =  new Node3D(p.x, p.y, angle + 0.5*M_PI, 1, 1, nPred_tmp_ptr, prim);
                            nPred_tmp_ptr = tmp_node;
                        }
                    }
                    goal.setPred(nPred_tmp_ptr);
                    return &goal;
                }
            }
            //充实open集6向搜索
            for(int i = 0; i < Node3D::dir; i ++)
            {
                node3D_tmp = nPred.createSuccessor(i, 1 / scale);   //i向步进1/scale搜索子节点
                nSucc = *node3D_tmp;
                iSucc = nSucc.setIdx(width, height);    //拿到子节点Idx
                //网格范围检测、节点通行性检测
                if(nSucc.isOnGrid(width, height) && m_map->isNodeTraversable(&nSucc))
                {
                    //未在close集中，且和父节点共格点可能需要更新
                    if(!m_nodes3D[iSucc].isClosed() || iPred == iSucc)
                    {
                        nSucc.updateG();    //更新子节点已付出代价
                        newG = nSucc.getG();
                        if(!m_nodes3D[iSucc].isOpen() || newG < m_nodes3D[iSucc].getG() || iPred == iSucc)
                        {
                            updateH(nSucc, goal);   //更新启发代价
                            //若nSucc与nPred同一格点且代价更大当废点处理
                            if(iPred == iSucc && nSucc.getC() > nPred.getC() + param::tieBreaker)
                            {
                                delete node3D_tmp;
                                continue;
                            }
                            //若nSucc与nPred同一格点且代价更小替代当前节点
                            else if(iPred == iSucc && nSucc.getC() <= nPred.getC() + param::tieBreaker)
                            {
                                nSucc.setPred(nPred.getPred());
                            }
                            if(nSucc.getPred() == nSucc) std::cout << "looping";    //原地踏步
                            nSucc.open();   //子节点标记为open
                            m_nodes3D[iSucc] = nSucc;   //存入m_nodes3D[]中
                            open.insert(m_nodes3D[iSucc]);  //置入open集
                            delete node3D_tmp;
                        }
                    }
                }
            }
        }
    }
    return nullptr;
}

//RS路径插值
void hybridAStar::interpolate(const ReedsShepp::pos *from, double t,
                         ReedsShepp::pos *state) const
{
    m_RS.interpolate(from, m_RS_path, t, state);
}