#ifndef AP_COMPONENT_HPP
#define AP_COMPONENT_HPP

#include "AP_interfaceProcess.hpp"
#include "Action_Planning.hpp"

using sysd::Time;

namespace actionPlanning
{

    class AP_Component : public sysd::PeriodicComponent
    {
    public:
        AP_Component() : sysd::PeriodicComponent("actionPlanning")
        {

        }
        ~AP_Component()
        {
            
        }
    }
}

#endif      //AP_COMPONENT_HPP