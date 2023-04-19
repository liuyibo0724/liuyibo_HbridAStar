#ifndef HYBRID_ASTAR_HYBRIDASTAR_H
#define HYBRID_ASTAR_HYBRIDASTAR_H
#include <vector>
#include <algorithm>
#include "node.h"
#include "CollisionDetection.h"
#include "ReedsSheppPath.h"
#include "smooth.h"

namespace HybridAStar
{
    bool compareNode3DSet(std::vector<Node3D> &path1, std::vector<Node3D> &path2);   //m_nodes3D_Set排序准则
    class hybridAStar
    {
    public:
        //显式构造函数传入地图数据
        explicit hybridAStar(CollisionDetection *map);
        //析构函数
        ~hybridAStar();
        //hybridAStar搜索路径的核心规划器，返回3D点列
        //（注意scale的意义不明也许是缩放尺度记得修改）
        Node3D* search_planner(Node3D &start, Node3D &goal, float scale = 0.5);
        //传统aStar规划器返回2D末点的已付出代价G
        //（注意scale的意义不明也许是缩放尺度记得修改）
        float aStar(Node2D &start, Node2D &goal, float scale = 1.0);
        void interpolate(const ReedsShepp::pos *from, float t,
                         ReedsShepp::pos *state) const; //RS路径插值
        //后半程是否用RS曲线射入goal成功
        bool isShootSuccess(){ return m_shootSuccess; }
        Node3D *m_nodes3D{};

        std::vector<std::vector<Node3D>> m_nodes3D_Set; //保存成品点列的队列
        bool sortNode3D_Set();                          //对保存成品点列的队列进行自定义排序，排序准则长度和段数升序；并对每一个成品点列进行倒序排列

    private:
        //更新启发代价H
        void updateH(Node3D &start, Node3D &goal);
        Node2D *m_nodes2D{};
        
        CollisionDetection *m_map{};
        ReedsShepp m_RS{param::RS_Scaling};
        ReedsShepp::ReedsSheppPath m_RS_path;
        bool m_shootSuccess;
        

    };
}
#endif      //HYBRID_ASTAR_HYBRIDASTAR_H