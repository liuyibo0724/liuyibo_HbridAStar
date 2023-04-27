#ifndef HYBRID_ASTAR_DYNAMICVORONOI_H
#define HYBRID_ASTAR_DYNAMICVORONOI_H
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <iostream>
#include <cmath>
#include <queue>
#include <unordered_map>
#include "vec2i.h"
#include "param.h"
#define MAXDIST 1000
#define INTPOINT IntPoint

namespace HybridAStar
{
    //2D向量类
    class Vector2D
    {
    public:
        inline Vector2D(const double x = 0, const double y = 0){ this->x = x; this->y = y; }//构造函数
        // 计算符重载
        inline Vector2D operator + (const Vector2D &cpr) const { return Vector2D(x + cpr.x, y + cpr.y); }
        inline Vector2D operator - (const Vector2D &cpr) const { return Vector2D(x - cpr.x, y - cpr.y); }
        inline Vector2D operator * (const double &k) const { return Vector2D(x * k, y * k); }
        inline Vector2D operator / (const double &k) const { return Vector2D(x / k, y / k); }
        inline Vector2D operator - () const { return Vector2D(-x, -y); }
        friend std::ostream& operator << (std::ostream &os, const Vector2D &cpr) { os << "(" << cpr.x << ", " << cpr.y << ")"; return os; }
        //向量长度
        double length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2)); }
        //向量长度平方
        double sqrlength() const { return x * x + y * y; }
        //向量点乘
        double dot(const Vector2D &cpr) const { return x * cpr.x + y * cpr.y; }
        //向量叉乘
        double cross(const Vector2D &cpr) const { return x * cpr.y - y * cpr.x; }
        //求与某向量正交的分量
        inline Vector2D ort(const Vector2D &cpr) const
        {
            Vector2D tmpSelf(this->x, this->y);
            Vector2D result = tmpSelf - cpr * tmpSelf.dot(cpr) / cpr.sqrlength();
            return result;
        }
        // 查询类函数
        inline double getX() const { return x; }
        inline double getY() const { return y; }

    private:
        double x;
        double y;
    };
    inline Vector2D operator * (double k, const Vector2D &cpr){ return (cpr * k); }
    
    class IntPoint
    {
    public:
        IntPoint() : x(0), y(0) {}
        IntPoint(int _x, int _y) : x(_x), y(_y) {}
        int x, y;
    };

    class BucketPrioQueue
    {
    public:
        //! Standard constructor
        /** Standard constructor. When called for the first time it creates a look up table
            that maps square distanes to bucket numbers, which might take some time...
        */
        BucketPrioQueue();//构造函数
        //! Checks whether the Queue is empty
        bool empty();//检验队列是否为空
        //! push an element
        void push(int prio, INTPOINT t);//插入元素
        //! return and pop the element with the lowest squared distance */
        INTPOINT pop();

    private:

        static void initSqrIndices();
        static std::vector<int> sqrIndices;//记录pri的位置，sqrIndices[pri]=index
        static int numBuckets;
        int count;//有效元素数量
        int nextBucket;
        std::vector<std::queue<INTPOINT> > buckets;
    };

//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
    class DynamicVoronoi
    {
    public:
        DynamicVoronoi();
        ~DynamicVoronoi();

        //! Initialization with an empty map 从空白地图开始构造
        void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);
        //! Initialization with a given binary map (false==free, true==occupied) 从Grid Map初始化
        void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

        //! add an obstacle at the specified cell coordinate
        void occupyCell(int x, int y); //在 (x,y) 位置标记障碍
        //! remove an obstacle at the specified cell coordinate
        void clearCell(int x, int y); //移除(x,y)位置的障碍
        //! remove old dynamic obstacles and add the new ones
        void exchangeObstacles(std::vector<INTPOINT> newObstacles);//用新的障碍信息替换旧的障碍信息

        //! update distance map and Voronoi diagram to reflect the changes
        void update(bool updateRealDist = true);//根据环境变化更新距离地图和Voronoi Diagram
        //! prune the Voronoi diagram
        void prune();//对Voronoi diagram剪枝
        void CollectVoronoiEdgePoints();//搜集voronoi边界点列
        Vec2i GetClosestVoronoiEdgePoint(Vector2D xi, double& closest_dis); //得到voronoi边界点列
        float voronoiField(int x, int y);   //计算voronoi场的值

        //! returns the obstacle distance at the specified location
        float getDistance(int x, int y);//返回(x,y)位置处的最近障碍的距离
        //! returns whether the specified cell is part of the (pruned) Voronoi graph
        bool isVoronoi(int x, int y);//检查(x,y)处是否为(剪枝后的)Voronoi graph的一部分
        //! checks whether the specficied location is occupied
        bool isOccupied(int x, int y);//检查(x,y)是否为占据状态
        //! write the current distance map and voronoi diagram as ppm file
        void visualize(const char* filename = "result.ppm");//将当前的距离地图和voronoi diagram写进ppm文件里

        //! returns the horizontal size of the workspace/map
        unsigned int getSizeX() {return sizeX;}//返回地图X向size
        //! returns the vertical size of the workspace/map
        unsigned int getSizeY() {return sizeY;}//返回地图Y向size

        // was private, changed to public for obstX, obstY
    public:
        struct dataCell {
            float dist;//距离
            char voronoi;//
            char queueing;
            int obstX;
            int obstY;
            bool needsRaise;
            int sqdist;
        };

//状态，枚举型
        typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
        typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
        typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
        typedef enum {pruned, keep, retry} markerMatchResult;

        // methods
        void setObstacle(int x, int y);//在(x,y)处设置障碍
        void removeObstacle(int x, int y);//移除(x,y)处障碍
        //
        inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
        //
        void recheckVoro();
        //更新并刷新以可视化
        void commitAndColorize(bool updateRealDist = true);
        inline void reviveVoroNeighbors(int& x, int& y);

//检查是否为占据状态
        inline bool isOccupied(int& x, int& y, dataCell& c);
        //标记匹配结果
        inline markerMatchResult markerMatch(int x, int y);

        // queues

        BucketPrioQueue open;
        std::queue<INTPOINT> pruneQueue;

        std::vector<INTPOINT> removeList;
        std::vector<INTPOINT> addList;
        std::vector<INTPOINT> lastObstacles;

        // maps
        int sizeY;
        int sizeX;
        dataCell** data;
        bool** gridMap;

        // parameters
        int padding;
        double doubleThreshold;

        double sqrt2;
        //  dataCell** getData(){ return data; }
        //voronoi边界点列
        std::vector<Vec2i> edge_points_;
        std::unordered_map<std::string, std::pair<Vec2i, float>> closest_edge_points_;
        //计算Vec2d或Vec2i点索引
        std::string ComputeIndex(const Vec2i& pi) const;
        std::string ComputeIndex(const Vector2D& pd) const;
    };
}


#endif //HYBRID_ASTAR_DYNAMICVORONOI_H
