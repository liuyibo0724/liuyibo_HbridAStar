#ifndef ACTION_PLANNING_HPP
#define ACTION_PLANNING_HPP

#include <stdlib.h>
#include "includes/planner.h"

namespace actionPlanning
{
    //ActionPlanner类的声明和定义
    class ActionPlanner
    {
    public:
        ActionPlanner() {}  //构造函数
        ~ActionPlanner() {} //析构函数
        bool init(unsigned char* data, int width, int height);        //初始化
        bool run();   //运行
    private:
        HybridAStar::planner *m_planner;
    };
}

#endif  //ACTION_PLANNING_HPP