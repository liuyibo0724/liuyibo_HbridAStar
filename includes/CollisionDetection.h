#ifndef HYBRID_ASTAR_COLLISIONDETECTION_H
#define HYBRID_ASTAR_COLLISIONDETECTION_H
#include <math.h>
#include <string.h>
#include <vector>
#include <set>
#include "node.h"

namespace HybridAStar
{
    class CollisionDetection
    {
    public:
        CollisionDetection(unsigned char* data, int width, int height);  //构造函数
        //1.连通性检查函数
        bool isNodeTraversable(Node2D* node);    //2D结点可否通行
        bool isNodeTraversable(Node3D* node);    //3D结点可否通行
        bool isConfigTraversable(float x, float y, float t);     //某位姿可否通行
        inline bool isInMap(int x, int y);       //点是否在图中
        //2.设置类函数
        void updateMap(unsigned char* data, int width, int height);     //读入新图
        void setCollisionLookup(float x, float y, float t);      //设置亚格子碰撞查询清单
        //3.查询类函数
        int getSize() const { return m_width * m_height; }      //图像素数
        int getWidth() const { return m_width; }    //查询图宽度
        int getHeight() const { return m_height; }  //查询图高度
    private:
        int m_width;    //图网格宽度
        int m_height;   //图网格高度
        unsigned char* m_map;   //读入的图片信息
        //网格内特定姿态的碰撞查询清单（集合set格式）
        param::poseConfig collisionLookup;      //[param::positions * param::headings];
    };
}


#endif //HYBRID_ASTAR_COLLISIONDETETION