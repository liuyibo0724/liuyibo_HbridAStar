#include "CollisionDetection.h"

using namespace HybridAStar;

//所给点是否在已知向量右侧
inline bool isPosRVec(param::relPos vec, param::relPos pos)
    return pos.x * vec.y - vec.x * pos.y >= 0;

//快速构造relPos
inline param::relPos mkRelPos(double x, double y)
{
    param::relPos result;
    result.x = x;
    result.y = y;
    return result;
}

//由位姿生成对应的collisionLookup亚格子碰撞查询队列
void HybridAStar::CollisionDetetion::setCollisionLookup(
    double x, 
    double y, 
    double t)
{
    double dx = 0.5 * param::width, dy = 0.5 * param::length;   //半车宽和半车长
    param::relPos pos_list[4];
    pos_list[0] = mkRelPos(x + dx*sin(t) + dy*cos(t), y - dx*cos(t) + dy*sin(t));
    pos_list[1] = mkRelPos(x + dx*sin(t) - dy*cos(t), y - dx*cos(t) - dy*sin(t));
    pos_list[2] = mkRelPos(x - dx*sin(t) - dy*cos(t), y + dx*cos(t) - dy*sin(t));
    pos_list[3] = mkRelPos(x - dx*sin(t) + dy*cos(t), y + dx*cos(t) + dy*sin(t));
    double minX = pos_list[0].x, minY = pos_list[0].y, 
            maxX = pos_list[0].x, maxY = pos_list[0].y;      //四个边角点的坐标极值
    for(int i = 1; i < 4; i ++)
    {
        maxX = pos_list[i].x > maxX ? pos_list[i].x : maxX;
        maxY = pos_list[i].y > maxY ? pos_list[i].y : maxY;
        minX = pos_list[i].x < minX ? pos_list[i].x : minX;
        minY = pos_list[i].y < minY ? pos_list[i].y : minY;
    }
    for(int i = (int)(ceil(minX)); i < (int)(floor(maxX)); i ++)
    {
        for(int j = (int)(ceil(minY)); j < (int)(floor(maxY)); j ++)
        {
            for(int k = 0; k < 4; k ++)
            {
                param::relPos temp_vec = 
                    mkRelPos(pos_list[(k + 1)%4].x - pos_list[k].x
                    , pos_list[(k + 1)%4].y - pos_list[k].y);
                param::relPos temp_pos = 
                    mkRelPos(x - pos_list[k].x, y - pos_list[k].y);
            }
        }
    }
}

//构造函数
HybridAStar::CollisionDetetion::CollisionDetetion(
    unsigned char* data, 
    int width, 
    int height)  
{
    this->m_map = new unsigned char[width * height];
    memcpy(m_map, data, width * height * sizeof(unsigned char));
    this->m_width = whidth;
    this->m_height = height;
}