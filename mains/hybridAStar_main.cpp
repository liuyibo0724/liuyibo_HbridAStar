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
    cv::resize(map_gray, map_gray, cv::Size(map_gray.cols/10, map_gray.rows/10));   //缩小黑白图片

    cv::Mat map_color
         = cv::imread("/home/liuyibo/liuyibo_HbridAStar/test_pictures/map1.png");
    cv::resize(map_color, map_color, cv::Size(map_color.cols/10, map_color.rows/10));   //缩小彩色图片
    
    CollisionDetection map_data(map_gray.data, map_gray.cols, map_gray.rows);
    hybridAStar planer(&map_data);
    Node3D start(8, 8, 0, 0, 0, nullptr);
    Node3D goal(160 * 2.5, 160 * 2.5, 1.5 * M_PI, 0, 0, nullptr);
    
    //hybridAStar粗搜索
    auto nSolution = planer.search_planner(start, goal, 0.2);
    auto tmp_show = nSolution;
    while(tmp_show != nullptr)
    {
        int x = (int)tmp_show->getX()*inv_resolution;
        int y = (int)tmp_show->getY()*inv_resolution;
        if(tmp_show->getPred() != nullptr) cv::line(map_color,cv::Point(y,x),cv::Point(tmp_show->getPred()->getY(),
                                    tmp_show->getPred()->getX()),cv::Scalar(0,0,255));
        tmp_show = tmp_show->getPred();
    }
    cv::imshow("rawHybridAStar_result",map_color);
    cv::waitKey(0);

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
    smoother.tracePath(nSolution);
    smoother.smoothPath(voronoi);
    auto smooth_path = smoother.getPath();
    for(auto pt:smooth_path)
    {
        int x = (int)pt.getX();
        int y = (int)pt.getY();
        if(pt.getPred() != nullptr) cv::line(map_color, cv::Point(y, x), cv::Point(pt.getPred()->getY(),
                                pt.getPred()->getX()), cv::Scalar(255, 0, 0));
    }
    cv::imshow("smooth_result",map_color);
    cv::waitKey(0);

    return 0;
}