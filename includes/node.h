#ifndef HYBRID_ASTAR_NODE_H
#define HYBRID_ASTAR_NODE_H
#include <cmath>
#include <corecrt_math_defines.h>
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
    int setIdx(const int &width) { this->idx = this->y*width + this->x; return idx; } //x, y顺序有疑问已修改
    //将节点加入open集
    void open() { this->o = true; }
    //将节点加入closed集
    void close() { this->c = true; }
    //重置节点，擦除open和closed标签
    void reset() { this->o = false; this->c = false; }
    //发现节点
    void discover() { this->d = true; }
    //设置父节点
    void setPred(Node2D *pred) { this->pred = pred; }

    //3.更新类函数
    //单步代价
    double singleMoveCost(Node2D *pred) const
    {
        return sqrt((x - pred->x)*(x - pred->x) + (y - pred->y)*(y - pred->y));
    }
    //更新已经付出的代价g
    void updateG() { g += singleMoveCost(this->pred); }
    //更新启发代价H
    void updateH(Node2D &goal) { h = singleMoveCost(&goal); }

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
    //x方向可能的移动步长
    const int dx[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
    //y方向可能的移动步长
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
    //参数构造函数
    Node3D(double x, double y, double t, double g, double h, Node3D* pred, int prim = 0)
    {
        this->x = x;
        this->y = y;
        this->t = t;
        this->g = g;
        this->h = h;
        this->pred = pred;
        this->prim = prim;
        this->o = false;
        this->c = false;
        this->idx = -1;
    }
    //默认构造函数
    Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}

    //1.查询类函数：
    //查询坐标和姿态角
    double getX() { return x; }
    double getY() { return y; }
    double getT() { return t; }
    //查询已经付出的和启发代价
    double getG() { return g; }
    double getH() { return h; }
    //查询总代价
    double getC() { return g + h; }
    //查询3D索引值
    int getIdx() { return idx; }
    //查询节点相关联的运动基元数
    int getPrim() { return prim; }
    //查询是否在open集
    bool isOpen() { return o; }
    //查询是否在closed集中
    bool isClosed() { return c; }
    //查询是否可以通过解析方法求得解
    bool isSolvable(const Node3D &goal) const
    {
        int cout = 0;
        for(int i = 0; i < 10; i ++)
        {
            double dx = std::abs(x - goal.x) / static_cast<double>(i);
            double dy = std::abs(y - goal.y) / static_cast<double>(i);
            if(std::pow(dx, 2) + std::pow(dy, 2) < param::dubinsShotDistance)
                ++cout;
        }
        return cout > 7;
    }
    //查询是否超出图网格范围
    bool isOnGrid(const int width, const int height) const
    {
        return x >= 0 && x < width && y >= 0 && y < height
        && (int)(t / param::deltaHeadingRad) >= 0
        && (int)(t / param::deltaHeadingRad) < param::headings;
    }
    //查询父节点
    Node3D* getPred() { return pred; }

    //2.设置类函数
    //设置节点相关运动基元数
    void setPrim(int p) { this->prim = p; }
    //设置坐标和姿态角
    void setX(double x) { this->x = x; }
    void setY(double y) { this->y = y; }
    void setT(double t) { this->t = t; }
    //设置已经付出的和启发代价
    void setG(double g) { this->g = g; }
    void setH(double h) { this->h = h; }
    //设置3D索引值
    int setIdx(int width, int height)
    {
        this->idx = 
            (int)(t / param::deltaHeadingRad) * width * height
            + (int)(y) * width + (int)(x);
        return idx;
    }
    //将节点加入open集
    void open() { this->o = true; this->c = false; }
    //将节点加入closed集
    void close() { this->o = false; this->c = true; }
    //设置父结点
    void setPred(Node3D* pred) { this->pred = pred; }
    //重置3D结点
    void reset()
    {
        x = 0;
        y = 0;
        t = 0;
        pred = nullptr;
        o = false;
        c = false;
    }

    //3.更新类函数
    //更新已经付出的代价g
    void updateG();

    //4.计算符重载类函数
    bool operator == (const Node3D &comparator) const;

    //5.创建某方向上子结点
    //创建子结点
    Node3D* createSuccessor(const int i, double inv_scale);
    //创建子结点方向数量
    static const int dir = 6;
    //x可能的移动步长
    const double dx[3] = { 0.7068582,   0.705224,   0.705224 };
    //y可能的移动步长
    const double dy[3] = { 0,        -0.0415893,     0.0415893 };
    //t可能的移动步长
    const double dt[3] = { 0,         0.1178097,    -0.1178097 };

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
};

}

#endif //HYBRID_ASTAR_NODE_H