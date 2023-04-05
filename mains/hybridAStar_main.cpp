#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include "node.h"
#include "hybridAStar.h"
#include "smooth.h"
#include "dynamicVoronoi.h"

using namespace HybridAStar;

int main()
{
    cv::Mat map_gray
         = cv::imread("/home/liuyibo/liuyibo_HbridAStar/test_pictures/map1.png", cv::IMREAD_GRAYSCALE);
    int inv_resolution = 1;
    cv::resize(map_gray, map_gray, cv::Size(map_gray.cols/25, map_gray.rows/25));   //缩小黑白图片

    cv::Mat map_color
         = cv::imread("/home/liuyibo/liuyibo_HbridAStar/test_pictures/map1.png");
    cv::resize(map_color, map_color, cv::Size(map_color.cols/25, map_color.rows/25));   //缩小彩色图片
    
    CollisionDetection map_data(map_gray.data, map_gray.cols, map_gray.rows);
    hybridAStar planer(&map_data);
    Node3D start(8, 8, 0.4 * M_PI, 0, 0, nullptr);
    Node3D goal(160 * 1, 220 * 1, 1.5 * M_PI, 0, 0, nullptr);

    //生图展示
    cv::imshow("raw_pic",map_color);
    cv::waitKey(0);
    
    //hybridAStar粗搜索
    auto nSolution = planer.search_planner(start, goal, 0.2);
    auto tmp_show = nSolution;


    //绘制voronoi图
    DynamicVoronoi voronoi;
    bool** binMap;  //二维数组
    binMap = new bool*[map_color.rows];
    for(int x = 0; x < map_color.rows; x ++) { binMap[x] = new bool[map_color.cols]; }
    for(int x = 0; x < map_color.rows; x ++)
    {
        for(int y = 0; y < map_color.cols; y ++)
        {
            binMap[x][y] = map_gray.data[y + x * map_color.cols] < 250;
        }
    }   //转化为二值地图
    voronoi.initializeMap(map_color.rows, map_color.cols, binMap);
    voronoi.update();

    //轨迹光顺
    Smoother smoother;
    
    Node3D tmp_node = *nSolution;
    while(tmp_node.pIdx != -1)
    {
        float x = tmp_node.getX();
        float y = tmp_node.getY();
        float t = tmp_node.getT();
        int prim = tmp_node.getPrim();
        smoother.m_path.push_back(Node3D(x, y, t, 0, 0, nullptr, prim));

        tmp_node = planer.m_nodes3D[tmp_node.pIdx];
    }
    //轨迹光顺前画线
    for(int j = 0; j < smoother.m_path.size() - 1; j ++)
    {
        cv::line(map_color, cv::Point(smoother.m_path[j].getY(), smoother.m_path[j].getX()),
                            cv::Point(smoother.m_path[j + 1].getY(), smoother.m_path[j + 1].getX()), cv::Scalar(0, 0, 255));
    }
    std::cout << "smoother.m_path.size() = " << smoother.m_path.size() << std::endl;
    cv::imshow("smooth_beform",map_color);
    cv::waitKey(0);

    // smoother.tracePath(tmp_show);
    smoother.smoothPath(voronoi);
    auto smooth_path = smoother.getPath();
    //轨迹光顺后画线
    for(int j = 0; j < smooth_path.size() - 1; j ++)
    {
        cv::line(map_color, cv::Point(smooth_path[j].getY(), smooth_path[j].getX()),
                            cv::Point(smooth_path[j + 1].getY(), smooth_path[j + 1].getX()), cv::Scalar(255, 0, 0));
    }
    std::cout << "smooth_path.size() = " << smooth_path.size() << std::endl;
    cv::imshow("smooth_result",map_color);
    cv::waitKey(0);

    for(int i = 0; i < map_color.rows; i ++) delete[] binMap[i];    //释放内存binMap
    delete[] binMap;                                                //释放内存binMap

    return 0;
}