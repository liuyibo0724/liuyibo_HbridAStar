#include "CollisionDetection.h"

using namespace HybridAStar;

//所给点是否在已知向量右侧
inline bool isPosRVec(param::relPos vec, param::relPos pos)
    { return pos.x * vec.y - vec.x * pos.y >= 0; }

//快速构造relPos
inline param::relPos mkRelPos(int x, int y)
{
    param::relPos result;
    result.x = x;
    result.y = y;
    return result;
}

//是否在图中
inline bool HybridAStar::CollisionDetection::isInMap(int x, int y)
    { return x >= 0 && x < m_height && y >= 0 && y < m_width; }

//由目标点在m_map中画出车位边界线
bool HybridAStar::CollisionDetection::drawParkingSpaceOnMap(Node3D goal)
{
    this->parkingSpaceProfile.clear();
    float x = goal.getX(), y = goal.getY(), t = goal.getT();
    if(!isInMap((int)x, (int)y)) return false;
    float dx = 0.5 * param::parkingSpaceWidth, dy = 0.5 * param::parkingSpaceLength;    //车位半宽度和半长度
    param::relPos pos_list[4];
    pos_list[0] = mkRelPos(x + dx*sin(t) + param::parkingFront2Rate*dy*cos(t), y - dx*cos(t) + param::parkingFront2Rate*dy*sin(t));
    pos_list[1] = mkRelPos(x + dx*sin(t) - param::parkingRear2Rate*dy*cos(t), y - dx*cos(t) - param::parkingRear2Rate*dy*sin(t));
    pos_list[2] = mkRelPos(x - dx*sin(t) - param::parkingRear2Rate*dy*cos(t), y + dx*cos(t) - param::parkingRear2Rate*dy*sin(t));
    pos_list[3] = mkRelPos(x - dx*sin(t) + param::parkingFront2Rate*dy*cos(t), y + dx*cos(t) + param::parkingFront2Rate*dy*sin(t));
    
    for(int i = 0; i < 3; i ++)
    {
        for(int j = 0; j < 101; j ++)
        {
            float per = (float)j / 100.f;
            int x_tmp = per * pos_list[i].x + (1.f - per) * pos_list[i + 1].x;
            int y_tmp = per * pos_list[i].y + (1.f - per) * pos_list[i + 1].y;
            if(isInMap(x_tmp, y_tmp)) parkingSpaceProfile.insert(mkRelPos(x_tmp, y_tmp));
        }
    }
    //绘制车位边界线
    for(auto ptr = parkingSpaceProfile.begin(); ptr != parkingSpaceProfile.end(); ptr ++)
    {
        m_map[ptr->x * m_width + ptr->y] = 0;
    }
    return true;
}

//由位姿生成对应的collisionLookup亚格子碰撞查询队列
void HybridAStar::CollisionDetection::setCollisionLookup(
    float x, 
    float y, 
    float t)
{
    float dx = 0.5 * param::width, dy = 0.5 * param::length;   //半车宽和半车长加上padding
    param::relPos pos_list[4];
    pos_list[0] = mkRelPos(x + dx*sin(t) + param::front2Rate*dy*cos(t), y - dx*cos(t) + param::front2Rate*dy*sin(t));
    pos_list[1] = mkRelPos(x + dx*sin(t) - param::rear2Rate*dy*cos(t), y - dx*cos(t) - param::rear2Rate*dy*sin(t));
    pos_list[2] = mkRelPos(x - dx*sin(t) - param::rear2Rate*dy*cos(t), y + dx*cos(t) - param::rear2Rate*dy*sin(t));
    pos_list[3] = mkRelPos(x - dx*sin(t) + param::front2Rate*dy*cos(t), y + dx*cos(t) + param::front2Rate*dy*sin(t));
    // float minX = pos_list[0].x, minY = pos_list[0].y, 
    //         maxX = pos_list[0].x, maxY = pos_list[0].y;      //四个边角点的坐标极值
    // for(int i = 1; i < 4; i ++)
    // {
    //     maxX = pos_list[i].x > maxX ? pos_list[i].x : maxX;
    //     maxY = pos_list[i].y > maxY ? pos_list[i].y : maxY;
    //     minX = pos_list[i].x < minX ? pos_list[i].x : minX;
    //     minY = pos_list[i].y < minY ? pos_list[i].y : minY;
    // }
    // for(int i = (int)(ceil(minX)); i < (int)(floor(maxX)); i ++)
    // {
    //     for(int j = (int)(ceil(minY)); j < (int)(floor(maxY)); j ++)
    //     {
    //         if(!isInMap(i, j)) continue;
    //         int count = 0;      //计数点在右侧的边数
    //         for(int k = 0; k < 4; k ++)
    //         {
    //             param::relPos temp_vec = 
    //                 mkRelPos(pos_list[(k + 1)%4].x - pos_list[k].x
    //                 , pos_list[(k + 1)%4].y - pos_list[k].y);
    //             param::relPos temp_pos = 
    //                 mkRelPos(i - pos_list[k].x, j - pos_list[k].y);
    //             if(!isPosRVec(temp_vec, temp_pos)) break;
    //             ++ count;
    //         }
    //         if(count == 4)
    //         {
    //             if(isInMap(i - 1, j - 1)) this->collisionLookup.insert(mkRelPos(i - 1, j - 1));
    //             if(isInMap(i - 1, j)) this->collisionLookup.insert(mkRelPos(i - 1, j));
    //             if(isInMap(i, j - 1)) this->collisionLookup.insert(mkRelPos(i, j - 1));
    //             if(isInMap(i, j)) this->collisionLookup.insert(mkRelPos(i, j));
    //             if(isInMap(i + 1, j + 1)) this->collisionLookup.insert(mkRelPos(i + 1, j + 1));
    //             if(isInMap(i + 1, j)) this->collisionLookup.insert(mkRelPos(i + 1, j));
    //             if(isInMap(i, j + 1)) this->collisionLookup.insert(mkRelPos(i, j + 1));
    //         }
    //     }
    // }

    for(int i = 0; i < 4; i ++)
    {
        for(int j = 0; j < 10; j ++)
        {
            float t = (float)j / 10.f;
            this->collisionLookup.insert(mkRelPos((int)(t * pos_list[i].x + (1.f - t) * pos_list[(i + 1) % 4].x),
                                                  (int)(t * pos_list[i].y + (1.f - t) * pos_list[(i + 1) % 4].y)));
        }
    }
}

//构造函数
HybridAStar::CollisionDetection::CollisionDetection(
    unsigned char* data, 
    int width, 
    int height)  
{
    this->m_map = new unsigned char[width * height];
    memcpy(m_map, data, width * height * sizeof(unsigned char));
    this->m_width = width;
    this->m_height = height;
}

//2D节点可否通行
bool HybridAStar::CollisionDetection::isNodeTraversable(Node2D* node)
{
    //取得网格坐标（整数）
    int x = node->getX();
    int y = node->getY();
    if(x < 0 ||x >= m_height || y < 0 || y >= m_width) return false;
    return m_map[node->getIdx()] > 250;   //只要单网格够亮就算可通行，无需看周围网格
}

//3D节点可否通行
bool HybridAStar::CollisionDetection::isNodeTraversable(Node3D* node)
{
    //取得网格坐标（整数）
    double x = node->getX();
    double y = node->getY();
    if(x < 0 ||x >= m_height || y < 0 || y >= m_width) return false;
    return m_map[(int)x * m_width + (int)y] > 250;   //只要单网格够亮就算可通行，无需看周围网格
}

//某位姿可否通行
bool HybridAStar::CollisionDetection::isConfigTraversable(float x, float y, float t)
{
    this->collisionLookup.clear();  //清空collisionLookup集合
    //由位姿生成对应的collisionLookup亚格子碰撞查询队列
    HybridAStar::CollisionDetection::setCollisionLookup(x, y, t);
    for(auto ptr = this->collisionLookup.begin(); ptr != this->collisionLookup.end(); ptr ++)
    {
        if(isInMap(ptr->x, ptr->y) && m_map[ptr->x * m_width + ptr->y] < 250) return false;
    }
    return true;
}


//读入新图
void HybridAStar::CollisionDetection::updateMap(unsigned char* data, int width, int height)
{
    delete m_map;
    m_width = width;
    m_height = height;
    m_map = new unsigned char[width * height];
    memcpy(m_map, data, width * height * sizeof(unsigned char));
}

//start和goal是否反转
bool HybridAStar::CollisionDetection::reverseOrNot(Node3D &start, Node3D &goal, DynamicVoronoi &voronoi)
{
    float x = start.getX(), y = start.getY(), t = start.getT();
    float dx = 0.5 * param::width, dy = 0.5 * param::length;   //半车宽和半车长
    param::relPos pos_list[4];
    pos_list[0] = mkRelPos(x + dx*sin(t) + param::front2Rate*dy*cos(t), y - dx*cos(t) + param::front2Rate*dy*sin(t));
    pos_list[1] = mkRelPos(x + dx*sin(t) - param::rear2Rate*dy*cos(t), y - dx*cos(t) - param::rear2Rate*dy*sin(t));
    pos_list[2] = mkRelPos(x - dx*sin(t) - param::rear2Rate*dy*cos(t), y + dx*cos(t) - param::rear2Rate*dy*sin(t));
    pos_list[3] = mkRelPos(x - dx*sin(t) + param::front2Rate*dy*cos(t), y + dx*cos(t) + param::front2Rate*dy*sin(t));

    float start_obsDst_sum = 0.f;
    for(int i = 0; i < 4; i ++) start_obsDst_sum += voronoi.getDistance(pos_list[i].x, pos_list[i].y);

    x = goal.getX();
    y = goal.getY();
    t = goal.getT();
    dx = 0.5 * param::width;
    dy = 0.5 * param::length;   //半车宽和半车长
    pos_list[0] = mkRelPos(x + dx*sin(t) + param::front2Rate*dy*cos(t), y - dx*cos(t) + param::front2Rate*dy*sin(t));
    pos_list[1] = mkRelPos(x + dx*sin(t) - param::rear2Rate*dy*cos(t), y - dx*cos(t) - param::rear2Rate*dy*sin(t));
    pos_list[2] = mkRelPos(x - dx*sin(t) - param::rear2Rate*dy*cos(t), y + dx*cos(t) - param::rear2Rate*dy*sin(t));
    pos_list[3] = mkRelPos(x - dx*sin(t) + param::front2Rate*dy*cos(t), y + dx*cos(t) + param::front2Rate*dy*sin(t));

    float goal_obsDst_sum = 0.f;
    for(int i = 0; i < 4; i ++) goal_obsDst_sum += voronoi.getDistance(pos_list[i].x, pos_list[i].y);

    return start_obsDst_sum > goal_obsDst_sum;//start_obsDst_sum > goal_obsDst_sum;
}