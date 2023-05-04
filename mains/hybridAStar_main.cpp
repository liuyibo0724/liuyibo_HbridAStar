#include "CollisionDetection.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include "node.h"
#include "hybridAStar.h"
#include "smooth.h"
#include "dynamicVoronoi.h"
#include "BSpline.h"

using namespace HybridAStar;

//DynamicVoronoi前处理（需要传入图片的黑白版和彩色版）
DynamicVoronoi DynamicVoronoi_Pretreat(cv::Mat &map_gray, cv::Mat &map_color)
{
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
    voronoi.prune();
    voronoi.CollectVoronoiEdgePoints();
    for(int i = 0; i < map_color.rows; i ++) delete[] binMap[i];    //释放内存binMap
    delete[] binMap;                                                //释放内存binMap

    return voronoi;
}

//画voronoi场图
void showVoronoiField(cv::Mat &map_gray, cv::Mat &map_color)
{
    DynamicVoronoi voronoi =  DynamicVoronoi_Pretreat(map_gray, map_color);
    for(int i = 0; i < voronoi.sizeX; i ++)
    {
        for(int j = 0; j < voronoi.sizeY; j ++)
        {
            if(!voronoi.isOccupied(i, j)) 
            {
                float rhoV = voronoi.voronoiField(i, j);
                map_color.at<cv::Vec3b>(i, j) = {(unsigned char)(100000 - 100000 * rhoV), 
                                                 (unsigned char)(100000 - 100000 * rhoV), 
                                                 (unsigned char)(100000 - 100000 * rhoV)};
            }
        }
    }
    cv::imshow("rhoV_x_y",map_color);
    cv::waitKey(0);
}

//Smoother.m_path记录溯源路径
void SmootherGet_m_path(Smoother &smoother, std::vector<Node3D *>::iterator &ptr, int goalIdx)
{
    smoother.m_path.clear();    //每次循环清空smoother中的路径
    Node3D tmp_node = (*ptr)[goalIdx];
    while(tmp_node.pIdx != -1)
    {
        float x = tmp_node.getX();
        float y = tmp_node.getY();
        // float t = (tmp_node.getT() < 0) ? (tmp_node.getT() + 2.f * M_PI) : tmp_node.getT();
        float t = tmp_node.getT();
        int prim = tmp_node.getPrim();
        smoother.m_path.push_back(Node3D(x, y, t, 0, 0, nullptr, prim));    //轨迹溯源
        // tmp_node = planer.m_nodes3D[tmp_node.pIdx];
        tmp_node = (*ptr)[tmp_node.pIdx];
    }
}

//绘制车体轮廓
void drawCarProfile(cv::Mat &map_color, Node3D &node3D)
{
    float dx = 0.5 * param::width, dy = 0.5 * param::length;   //半车宽和半车长
    cv::Point pos_list[4];
    float x = node3D.getX(), y = node3D.getY(), t = node3D.getT();
    pos_list[0] = cv::Point(y - dx*cos(t) + param::front2Rate*dy*sin(t), x + dx*sin(t) + param::front2Rate*dy*cos(t));
    pos_list[1] = cv::Point(y - dx*cos(t) - param::rear2Rate*dy*sin(t), x + dx*sin(t) - param::rear2Rate*dy*cos(t));
    pos_list[2] = cv::Point(y + dx*cos(t) - param::rear2Rate*dy*sin(t), x - dx*sin(t) - param::rear2Rate*dy*cos(t));
    pos_list[3] = cv::Point(y + dx*cos(t) + param::front2Rate*dy*sin(t), x - dx*sin(t) + param::front2Rate*dy*cos(t));
    for(int i = 0; i < 4; i ++) cv::line(map_color, pos_list[i], pos_list[(i + 1)%4], cv::Scalar(0, 0, 0));  //画车框
}

//绘制车体轮廓运动的动画
void animaCarProfile(std::vector<std::pair<HybridAStar::Node3D, float>> BSplinePath, cv::Mat &map_color)
{
    int i = 0;
    for(auto tmp_pair : BSplinePath)
    {
        ++ i;
        auto map_color_clone = map_color.clone();
        drawCarProfile(map_color_clone, tmp_pair.first);
        cv::imshow("animation", map_color_clone);
        if(i == BSplinePath.size()) cv::waitKey(0);
        else cv::waitKey(50);
    }
}

//绘制线段切向指向线
void drawTangentLine(cv::Mat &map_color, Node3D &node3D)
{
    float x = node3D.getX(), y = node3D.getY(), t = node3D.getT();
    cv::line(map_color,cv::Point(y, x), cv::Point(y + 40 * sinf(t), x + 40 * cos(t)),cv::Scalar(0,0,0));  //画线段指向切线
}

//画碰撞检测区域
void drawCollisionLookup(cv::Mat &map_color, CollisionDetection &map_data, Node3D &node3D)
{
    map_data.isConfigTraversable(node3D.getX(), node3D.getY(), node3D.getT());  //更新map_data.collisionLookup
    auto CollisionLookup = map_data.getCollisionLookup();
    for(auto ptr = CollisionLookup.begin(); ptr != CollisionLookup.end(); ptr ++)
        if(map_data.isInMap((*ptr).x, (*ptr).y)) map_color.at<cv::Vec3b>((*ptr).x, (*ptr).y) = {0, 255, 0};  //画碰撞检测区域
}

//画碰撞检测区域
void drawParkingSpaceProfile(cv::Mat &map_color, CollisionDetection &map_data, Node3D &goal)
{
    map_data.drawParkingSpaceOnMap(goal);
    auto ParkingSpaceProfile = map_data.getParkingSpaceProfile();
    for(auto ptr = ParkingSpaceProfile.begin(); ptr != ParkingSpaceProfile.end(); ptr ++)
        if(map_data.isInMap((*ptr).x, (*ptr).y)) map_color.at<cv::Vec3b>((*ptr).x, (*ptr).y) = {0, 0, 0};  //画碰撞检测区域
}

int main()
{
    cv::Mat map_gray
         = cv::imread("/home/liuyibo/liuyibo_HbridAStar/test_pictures/map7.png", cv::IMREAD_GRAYSCALE);
    int inv_resolution = 1;
    cv::resize(map_gray, map_gray, cv::Size(map_gray.cols/25, map_gray.rows/25));   //缩小黑白图片

    cv::Mat map_color
         = cv::imread("/home/liuyibo/liuyibo_HbridAStar/test_pictures/map7.png");
    cv::resize(map_color, map_color, cv::Size(map_color.cols/25, map_color.rows/25));   //缩小彩色图片

    Node3D start(100, 50, 0.5 * M_PI, 0, 0, nullptr);
    Node3D goal(300 * 1, 320 * 1, 1. * M_PI, 0, 0, nullptr);
    CollisionDetection map_data(map_gray.data, map_gray.cols, map_gray.rows);                                  
    // drawParkingSpaceProfile(map_color, map_data, goal);     //在map_gray上画车位边界线
    hybridAStar planer(&map_data);
    int goalIdx = goal.setIdx(map_data.getWidth(), map_data.getHeight());   //拿到goal的idx索引

    //生图展示
    cv::imshow("raw_pic",map_color);
    cv::waitKey(0);
    
    //hybridAStar粗搜索
    auto nSolution = planer.search_planner(start, goal, 0.2);
    auto tmp_show = nSolution;
    planer.sortNode3D_Set();

    //DynamicVoronoi预处理
    DynamicVoronoi voronoi = DynamicVoronoi_Pretreat(map_gray, map_color);

    //轨迹光顺
    Smoother smoother;

    //BSpline曲线拟合
    BSpline::BSpline_referenceLine referenceLine;

    // for(auto ptr = planer.m_nodes3D_set.begin(); ptr < planer.m_nodes3D_set.end(); ptr ++)
    for(auto ptr = planer.m_nodes3D_Set.begin(); ptr < planer.m_nodes3D_Set.end(); ptr ++)
    {
        // SmootherGet_m_path(smoother, ptr, goalIdx);     //smoother.m_path记录溯源路径
        smoother.m_path = *ptr;
        //轨迹光顺前画线
        for(int j = 0; j < smoother.m_path.size() - 1; j ++)
        {
            map_color.at<cv::Vec3b>(smoother.m_path[j].getX(), smoother.m_path[j].getY()) = {0, 0, 255};
            // cv::line(map_color, cv::Point(smoother.m_path[j].getY(), smoother.m_path[j].getX()),
            //                     cv::Point(smoother.m_path[j + 1].getY(), smoother.m_path[j + 1].getX()), cv::Scalar(0, 0, 255));
        }
        std::cout << "smoother.m_path.size() = " << smoother.m_path.size() << std::endl;
        cv::imshow("smooth_before",map_color);
        cv::waitKey(0);

        // smoother.tracePath(tmp_show);
        smoother.smoothPath(voronoi);
        auto smooth_path = smoother.getPath();
        //轨迹光顺后画线
        for(int j = 0; j < smooth_path.size() - 1; j ++)
        {
            // cv::line(map_color, cv::Point(smooth_path[j].getY(), smooth_path[j].getX()),
            //                     cv::Point(smooth_path[j + 1].getY(), smooth_path[j + 1].getX()), cv::Scalar(255, 0, 0));
            map_color.at<cv::Vec3b>(smooth_path[j].getX(), smooth_path[j].getY()) = {255, 0, 0};

            float dx = 0.5 * param::width, dy = 0.5 * param::length;   //半车宽和半车长
            cv::Point pos_list[4];
            float t = smooth_path[j].getT();

            // drawCarProfile(map_color, smooth_path[j]);      //绘制车体轮廓
            // drawTangentLine(map_color, smooth_path[j]);     //画线段指向切线
        }
        // auto real_goal = planer.getReverseOrNot() ? start : goal;     //真goal考虑start和goal的调换
        // drawCollisionLookup(map_color, map_data, real_goal);          //画碰撞检测区域

        std::cout << "smooth_path.size() = " << smooth_path.size() << std::endl;
        cv::imshow("smooth_result",map_color);
        cv::waitKey(0);   

        //BSpline拟合
        referenceLine.clearRawPath();
        referenceLine.clearBSplinePath();
        referenceLine.setRawPath(smooth_path); 
        referenceLine.fit();
        auto BSplinePath = referenceLine.getBSplinePath();
        for(int j = 0; j < BSplinePath.size() - 1; j ++)
        {
            //前进绿色，停驻黑色，后退红色
            cv::Vec3b color;
            if(BSplinePath[j].second > 0) color = {0, (unsigned char)(255.f * BSplinePath[j].second / 28.f), 0};
            else color = {0, 0, (unsigned char)(-255.f * BSplinePath[j].second / 28.f)};
            map_color.at<cv::Vec3b>(BSplinePath[j].first.getX(), BSplinePath[j].first.getY()) = color;  //画速度大小
            // drawCarProfile(map_color, BSplinePath[j].first);      //绘制车体轮廓
        }
        cv::imshow("BSpline_result",map_color);
        cv::waitKey(0); 

        animaCarProfile(BSplinePath, map_color);    //绘制动画
    }

    return 0;
}