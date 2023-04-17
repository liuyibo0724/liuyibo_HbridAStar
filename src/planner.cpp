#include "planner.h"

using namespace HybridAStar;

//planner构造函数
planner::planner(unsigned char* data, int width, int height)
{
    m_map_data = new CollisionDetection(data, width, height);
    m_planner = new hybridAStar(m_map_data);
    m_voronoi = new DynamicVoronoi();
    m_referenceLine = new BSpline::BSpline_referenceLine();

    bool** binMap;  //二维数组
    binMap = new bool*[height];
    for(int x = 0; x < height; x ++) { binMap[x] = new bool[width]; }
    for(int x = 0; x < height; x ++)
    {
        for(int y = 0; y < width; y ++)
        {
            binMap[x][y] = data[y + x * width] < 250;
        }
    }   //转化为二值地图
    m_voronoi->initializeMap(height, width, binMap);
    m_voronoi->update();
    m_voronoi->prune();
    m_voronoi->CollectVoronoiEdgePoints();
    for(int i = 0; i < height; i ++) delete[] binMap[i];    //释放内存binMap
    delete[] binMap;
}

//规划核心函数
void planner::plan()
{
    if(!isStartSet || !isGoalSet)   //检验m_start和m_goal是否正确设置
    {
        std::cerr << "error !!" << " start state : " << isStartSet
                                << " goal state : " << isGoalSet
                                << std::endl;
        return;
    }
    m_planner->search_planner(m_start, m_goal, 0.2);    //hybridAStar路径搜索
    for(auto ptr = m_planner->m_nodes3D_Set.begin(); ptr < m_planner->m_nodes3D_Set.end(); ptr ++)
    {
        m_smoother->m_path = *ptr;

        m_smoother->smoothPath(*m_voronoi);
        auto smooth_path = m_smoother->getPath();

        //BSpline拟合
        m_referenceLine->clearRawPath();
        m_referenceLine->clearBSplinePath();
        m_referenceLine->setRawPath(smooth_path); 
        m_referenceLine->fit();
        m_BSplinePaths.push_back(m_referenceLine->getBSplinePath());
    }
    return;
}