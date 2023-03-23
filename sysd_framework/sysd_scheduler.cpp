#include "sysd_framework/sysd_scheduler.hpp"
#include "sysd_framework/sysd_log.hpp"

namespace sysd
{
    Scheduler Scheduler::m_scheduler;
    sysd::LogLevel g_logLevel = sysd::LogLevel::Info;
}