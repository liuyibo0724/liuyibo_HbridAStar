#ifndef __SYSD_SCHEDULER_HPP_
#define __SYSD_SCHEDULER_HPP_

#include "sysd_framework/common.hpp"
#include "sysd_framework/sysd_component.hpp"

#include "../license/license_checker.hpp"

#include <signal.h>
#include <sched.h>
// using namespace eprosima::fastdds::dds;

namespace sysd
{
/**
 * @brief class providing scheduling funtionalities of a sysd process, such as configing and scheduling 
 * of sysd periodic components and thread components, runtime statistics, process exiting. 
 * A process using sysd should instantiate one Scheduler.
 * 
 */
class Scheduler
{
private:
    /**
     * @brief Construct a new Scheduler object.
     * 
     */
    Scheduler()
    {
    }
    ~Scheduler()
    {
        for (auto scheduler : m_periodicSchedulers)
        {
            for (auto component : scheduler.components)
            {
                delete component;
            }
            if (scheduler.thread != nullptr)
            {
                delete scheduler.thread;
            }
        }
        for (auto component : m_threadComponents)
        {
            delete component;
        }
    }
   
    struct PeriodicScheduler
    {
        const char *name;
        uint32_t period_ms;
        std::vector<PeriodicComponent *> components;
        std::thread *thread;
    };

    static void periodicSchedulerThreadMain(PeriodicScheduler *scheduler)
    {
        // set thread name
        prctl(PR_SET_NAME, scheduler->name);
        for (PeriodicComponent *component : scheduler->components)
        {
            component->init();
        }
        sysd::Rate loopRate(scheduler->period_ms);
        // double tick_run;
        for (uint32_t i = 0; sysd::Scheduler::OK(); i++)
        {   
            for (PeriodicComponent *component : scheduler->components)
            {
                sysd::Time   tick = sysd::now();
                component->run(i);
                sysdPrintInfo("component \"{}.{}\".run() took {:04.3f} ms",
                              scheduler->name, component->getName(), (sysd::now() - tick).to_ns() / 1000000.0);
                // tick_run = ( sysd::now() - tick ).to_ns() / 1000000.0;
            }

            // sysd::Time tick_loop = sysd::now();
            loopRate.sleep();
            //sysdPrintWarn("\t\t\tick_loop \"{}\".run() took {:04.3f} ms",
                          //scheduler->name,  (sysd::now() - tick_loop).to_ns() / 1000000.0);
            if(loopRate.cycleTime() >= Time(scheduler->period_ms/1000.0))
            {
                sysdPrintWarn("-----------------------\"{}\" period took {:04.3f} ms(warn)---------------------",
                          scheduler->name, loopRate.cycleTime().to_ns() / 1000000.0);
                //sysdPrintWarn("\t\t\tick_loop \"{}\".run() took {:04.3f} ms",
                              //scheduler->name,  (sysd::now() - tick_loop).to_ns() / 1000000.0);
                //sysdPrintWarn("\t\t\tick_run \"{}\".run() took {:04.3f} ms",
                              //scheduler->name,  tick_run);
            }
            else
            {
                sysdPrintInfo("-----------------------\"{}\" period took {:04.3f} ms---------------------",
                          scheduler->name, loopRate.cycleTime().to_ns() / 1000000.0);
            }
            #ifdef J3_COMPONENTS_BUILD
                sysdPrintWarn("--\"{}\" period took {:04.3f} ms",
                          scheduler->name, loopRate.cycleTime().to_ns() / 1000000.0);
            #endif
            
        }
    }

    static void interruptHandler(int signal)
    {
        fprintf(stderr, "\ninterruptHandler\n");
        Scheduler::requestStop();
    }

    volatile bool m_shutting_down;
    std::vector<PeriodicScheduler> m_periodicSchedulers;
    std::vector<ThreadComponent *> m_threadComponents;

    static Scheduler m_scheduler;

public:
    static Scheduler *getInstance()
    {
        return &Scheduler::m_scheduler;
    }

    /**
     * @brief add a new periodic scheduler configuration to the scheduler. 
     * for each periodic scheduler configuration, the scheduler will create a new thread to implement the scheduling requirements.
     * after addPeriodicSchedulerConfig(), user can call addPeriodicComponent().
     * 
     * @param name Periodic scheduler name, should be unique
     * @param period_ms  the period in milliseconds for scheduler to periodically call run()
     * @return true 
     * @return false 
     */
    static bool addPeriodicSchedulerConfig(const char *name, uint32_t period_ms)
    {
        PeriodicScheduler scheduler;
        scheduler.name = name;
        scheduler.period_ms = period_ms;
        Scheduler::m_scheduler.m_periodicSchedulers.push_back(scheduler);
        return true;
    }
    /**
     * @brief add a new periodic component to the last added periodic scheduler config. 
     * 1 periodic scheduler config can have 0 or more periodic components.
     * 
     * @param component pointer to PeriodicComponent to add. 
     * This class will be deleted in ~Scheduler(), so user should not delete it directly, 
     * as a result component instance definition on stack is NOT recommended.
     * @return true 
     * @return false 
     */
    static bool addPeriodicComponent(PeriodicComponent *component)
    {
        if (Scheduler::m_scheduler.m_periodicSchedulers.size() == 0)
        {
            sysdPrintError("addPeriodicComponent fail: no PeriodicSchedulerConfig");
            sysd::handleFatalError();
            return false;
        }
        if (component == nullptr)
        {
            sysdPrintError("addPeriodicComponent fail: component is nullptr");
            sysd::handleFatalError();
            return false;
        }
        Scheduler::m_scheduler.m_periodicSchedulers.back().components.push_back(component);
        sysdPrintDebug("PeriodicComponent({}) added to '{}', period: {:d}ms\n",
                      component->getName(),
                      Scheduler::m_scheduler.m_periodicSchedulers.back().name,
                      Scheduler::m_scheduler.m_periodicSchedulers.back().period_ms);
        return true;
    }
    /**
     * @brief add a new thread component to the scheduler. 
     * for each thread component, the scheduler will create a new thread to schedule the component
     * 
     * @param component pointer to ThreadComponent to add. 
     * This class will be deleted in ~Scheduler(), so user should not delete it directly, 
     * as a result component instance definition on stack is NOT recommended.
     * @return true 
     * @return false 
     */
    static bool addThreadComponent(ThreadComponent *component)
    {
        if (component == nullptr)
        {
            sysdPrintError("addThreadComponent fail: component is nullptr");
            sysd::handleFatalError();
            return false;
        }
        Scheduler::m_scheduler.m_threadComponents.push_back(component);
        return true;
    }
    /**
     * @brief start the sysd scheduler
     * 
     * @return true 
     * @return false 
     */
    static bool start()
    {
        // wait for inputPins and outputPins connected
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        #ifdef J3_COMPONENTS_BUILD
            License::license_checker checker;
            checker.run();
        #endif

        for (auto scheduler = Scheduler::m_scheduler.m_periodicSchedulers.begin();
             scheduler != Scheduler::m_scheduler.m_periodicSchedulers.end(); ++scheduler)
        {
            scheduler->thread = new std::thread(&Scheduler::periodicSchedulerThreadMain, &(*scheduler));
            if (scheduler->thread == nullptr)
            {
                sysdPrintError("unexpected: new std::thread fail");
                sysd::handleFatalError();
            } else {
                pthread_setname_np(scheduler->thread->native_handle() , scheduler->name);
            }
        }

        for (ThreadComponent *component : Scheduler::m_scheduler.m_threadComponents)
        {
            component->start();
        }

        signal(SIGINT, sysd::Scheduler::interruptHandler);
        return true;
    }
    /**
     * @brief request to stop the scheduler. 
     * after this call, use should call waitForAllThreads() to wait for all threads of sysd to finish.
     * 
     * @return true 
     * @return false 
     */
    static bool requestStop()
    {
        Scheduler::m_scheduler.m_shutting_down = true;
        return true;
    }
    /**
     * @brief wait for all threads of sysd to finish.
     * 
     * @return true 
     * @return false 
     */
    static bool waitForAllThreads()
    {
        for (ThreadComponent *component : Scheduler::m_scheduler.m_threadComponents)
        {
            component->join();
        }
        for (auto scheduler = Scheduler::m_scheduler.m_periodicSchedulers.begin();
             scheduler != Scheduler::m_scheduler.m_periodicSchedulers.end(); ++scheduler)
        {
            scheduler->thread->join();
        }

        return true;
    }

    /**
     * @brief static function to check if sysd is ok. 
     * sysd is ok from the beginning, and not any more once requestStop() is called.
     * 
     * @return true 
     * @return false 
     */
    static bool OK()
    {
        return !Scheduler::m_scheduler.m_shutting_down;
    }
};

} // namespace sysd
#endif
