#ifndef HYBRID_ASTAR_NODE_H
#define HYBRID_ASTAR_NODE_H
#include <cmath>
#include "param.h"

namespace HybridAStar
{

class Node2D
{
public:
    //参数构造函数
    Node2D(int x, int y, double g, double h, Node2D* pred)
    {
        this->x = x;
        this->y = y;
        this->g = g;
        this->h = h;
        this->o = false;
        this->c = false;
        this->d = false;
        this->pred = pred;
        this->idx = -1;
    }
    //默认构造函数
    Node2D(): Node2D(0, 0, 0, 0, nullptr) {}

    //1.查询类函数：
    //查询x坐标
    int getX() const { return x; }
    //查询y坐标
    int getY() const { return y; }
    //查询已经付出的代价
    double getG() const { return g; }
    //查询启发代价
    double getH() const { return h; }
    //查询总代价
    double getC() const { return g + h; }
    //查询2D索引值
    int getIdx() const { return idx; }
    //查询是否在open集中
    bool isOpen() const { return o; }
    //查询是否在closed集中
    bool isClosed() const { return c; }
    //查询是否被发现
    bool isDiscovered() const { return d; }
    //查询是否超出图网格范围
    bool isOnGrid(const int width, const int height) const
    {
        return x >= 0 && x < height && y >= 0 && y < width;
    }
    //查询父节点
    Node2D* getPred() const { return pred; }

    //2.设置类函数
    //设置x坐标
    void setX(const int &x) { this->x = x; }
    //设置y坐标
    void setY(const int &y) { this->y = y; }
    //设置已经付出的代价
    void setG(const double &g) { this->g = g; }
    //设置启发代价
    void setH(const double &h) { this->h = h; }
    //设置2D索引值
    int setIdx(const int &width) { this->idx = this->x*width + this->y; return idx; }
    //将节点加入open集
    void open() { this->o = true; }
    //将节点加入closed集
    void close() { this->c = true; }
    //重置节点，擦除open和closed标签
    void reaet() { this->o = false; this->c = false; }
    //发现节点
    void discover() { this->d = true; }
    //设置父节点
    void setPred(Node2D *pred) { this->pred = pred; }

    //3.更新类函数
    //单步代价
    double singleMoveCost(Node2D *pred) const
    {
        return sqrt((x - this->x)*(x - this->x) + (y - this->y)*(y - this->y));
    }
    //更新已经付出的代价g
    void updateG() { g += singleMoveCost(this->pred); }
    //更新启发代价H
    void updateH(const Node2D goal) { h = singleMoveCost(&goal); }

    //4.计算符重载类函数
    //重载等号
    bool operator == (const Node2D &comparator) const
    {
        return this->x == comparator.x && this->y == comparator.y;
    }
    //阈值内的相等判别
    bool equal(const Node2D &comparator, double scale) const
    {
        return std::abs(this->x - comparator.x) < 1 / scale
            && std::abs(this->y - comparator.y) < 1 / scale;
    }

    //5.创建某方向上的子结点
    //创建子结点
    Node2D* createSuccessor(const int i, double inv_scale);
    //可能的方向
    static const int dir = 8;
    //x方向可能的方向
    const int dx[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
    //y方向可能的方向
    const int dy[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

private:
    //坐标
    int x;
    int y;
    //已经付出的代价
    double g;
    //启发代价
    double h;
    //2D索引值
    int idx;
    //是否在open集中
    bool o;
    //是否在closed集中
    bool c;
    //是否被发现
    bool d;
    //父节点
    Node2D* pred;
};

class Node3D
{
public:

private:
    //坐标
    double x;
    double y;
    double t;   //车辆姿态角度
    //已经付车的代价
    double g;
    //启发代价
    double h;
    //3D索引值
    int idx;
    //是否在open集中
    bool o;
    //是否在closed集中
    bool c;
    //节点的运动基元
    int prim;
    //父节点
    Node3D* pred;

}

}

#endif //HYBRID_ASTAR_NODE_H