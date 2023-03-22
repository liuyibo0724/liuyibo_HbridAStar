#include <corecrt_math_defines.h>
#include <iostream>
#include <boost/heap/binomial_heap.hpp>
#include "hybridAStar.h"

using namespace HybridAStar;
#define PI_SQR M_PI * M_PI

//Node2D点升序排列
class CompareNode2D
{
public:
    bool operator()(const Node2D &node1, const Node2D &node2)
    {
        return node1.getC() < node2.getC();
    }
};

//Node3D点升序排列
class CompareNode3D
{
public:
    bool operator()(const Node3D &node1, const Node3D &node2)
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