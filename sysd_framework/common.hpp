#ifndef __SYSD_COMMON_HPP_
#define __SYSD_COMMON_HPP_

#include <fastdds/rtps/common/Time_t.h>
#include "idl_generated/TimestampPubSubTypes.h"

#include "sysd_framework/sysd_log.hpp"

#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>

namespace sysd
{

inline void handleFatalError()
{
    sysdPrintError("handleFatalError: exit");
    sleep(1);
    exit(-1);
    //abort(); core dump
}

/**
 * @brief Time is similar to ROS Time. support >/</==/+/-/= operateors. 
 * sysd::Time start = sysd::now();
 * sysdPrintInfo("start time={}\n", sysd::toTimeString(start).c_str())
 * refer to https://fast-dds.docs.eprosima.com/en/latest/fastdds/api_reference/rtps/common/Time_t/fastrtps_Time_t.html
 */
using Time = eprosima::fastrtps::rtps::Time_t;
// using Time = eprosima::fastrtps::Time_t; //there is bug in nanosec operator-

/**
 * @brief Get the current time as a sysd::Time instance
 * @return Time 
 */
inline Time now()
{
    Time t;
    Time::now(t);
    return t;
}

inline Timestamp toTimestamp(Time t)
{
    Timestamp timestamp;
    timestamp.second = t.seconds();
    timestamp.nanosecond = t.nanosec();
    return timestamp;
}

inline Time fromTimestamp(Timestamp timestamp)
{
    Time t_time;
    t_time.seconds(timestamp.second);
    t_time.nanosec(timestamp.nanosecond);
    return t_time;
    // return Time(timestamp.second, timestamp.nanosecond);
}

/**
 * @brief return a time string of given time, like "18:00:00.000"
 * @param time 
 * @return std::string 
 */
inline std::string toTimeString(const Time &time)
{
    time_t t = time.seconds();
    std::stringstream ss;
    ss << std::put_time(std::localtime(&(t)), "%T") << "." << std::setw(3) << std::setfill('0') << time.nanosec() / 1000000;
    // sysdPrintInfo("toTimeString:{}", ss.str().c_str());
    return ss.str();
}

inline uint32_t getDomainId()
{
    char *domain_env = getenv("DDS_DOMAIN_ID");
    return (domain_env == nullptr) ? 0 : atoi(domain_env);
}

/**
 * @brief Class to help run loops at a desired period. 
 * Similar to ros::Rate, refer to http://docs.ros.org/diamondback/api/rostime/html/classros_1_1Rate.html
 * 
 */
class Rate
{
public:
    /**
     * @brief Construct a new Rate object
     * 
     * @param cycleTime_ms The desired period to run at in ms
     */
    Rate(uint32_t cycleTime_ms)
        : start_(sysd::now()), actual_cycle_time_(0.0)
    {
        expected_cycle_time_.nanosec(cycleTime_ms * 1000000);
    }

    /**
     * @brief Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
     * 
     */
    void sleep()
    {
        Time expected_end = start_ + expected_cycle_time_;
        Time actual_end = sysd::now();
        // detect backward jumps in time
        if (actual_end < start_)
        {
            expected_end = actual_end + expected_cycle_time_;
        }
        //calculate the time we'll sleep for
        Time sleep_time = expected_end - actual_end;
        //set the actual amount of time the loop took in case the user wants to know
        actual_cycle_time_ = actual_end - start_;
        //make sure to reset our start time
        start_ = expected_end;
        //if we've taken too much time we won't sleep
        if (sleep_time <= Time(0.0))
        {
            // if we've jumped forward in time, or the loop has taken more than a full extra
            // cycle, reset our cycle
            if (actual_end > expected_end + expected_cycle_time_)
            {
                start_ = actual_end;//->expected_end + expected_cycle_time_?
            }
            return;
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time.to_ns()));
        return;
    }
    /**
     * @brief Sets the start time for the rate to now.
     * 
     */
    void reset()
    {
        start_ = sysd::now();
    }
    /**
     * @brief Get the actual run time of a cycle from start to sleep.
     * 
     * @return Time The runtime of the cycle
     */
    Time cycleTime() { return actual_cycle_time_; }
    /**
     * @brief Get the expected cycle time -- one passed in to the constructor.
     * 
     * @return Time 
     */
    Time expectedCycleTime() { return expected_cycle_time_; }

private:
    Time start_;
    Time expected_cycle_time_;
    Time actual_cycle_time_;
};

} // namespace sysd

#endif