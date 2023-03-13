#ifndef HYBRID_ASTAR_NODE_H
#define HYBRID_ASTAR_NODE_H
#include <cmath>
#include "param.h"

namespace HybridAStar
{

class Node2D
{
public:
//默认构造函数
Node2D();
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

}

#endif //HYBRID_ASTAR_NODE_H