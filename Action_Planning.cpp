#include "Action_Planning.hpp"

using namespace actionPlanning;

bool ActionPlanner::init(unsigned char* data, int width, int height)
{
    m_planner = new HybridAStar::planner(data, width, height);
    return true;
}

bool ActionPlanner::run()
{
    
}