#include "sysd_framework/sysd_scheduler.hpp"

#include "AP_Component.hpp"

int main(int argc, char **argv)
{
    // /proc/sys/kernel/sched_rr_timeslice_ms = 100ms(Ubuntu)
    // sched_rt_runtime_us=950ms/1000ms
    printf("sched_getscheduler=%d\n", sched_getscheduler(0));

    sysd::LogLevel logLevel = sysd::LogLevel::Warn;
    if (argc > 1)
    {
        if (strcmp(argv[1], "warn") == 0)
        {
            printf("LogLevel-> warn\n");
            logLevel = sysd::LogLevel::Warn;
        }
        else if (strcmp(argv[1], "error") == 0)
        {
            printf("LogLevel-> error\n");
            logLevel = sysd::LogLevel::Error;
        }
        else if (strcmp(argv[1], "info") == 0)
        {
            printf("LogLevel-> info\n");
            logLevel = sysd::LogLevel::Info;
        }
        else if (strcmp(argv[1], "debug") == 0)
        {
            printf("LogLevel-> debug\n");
            logLevel = sysd::LogLevel::Debug;
        }
        else if (strcmp(argv[1], "trace") == 0)
        {
            printf("LogLevel-> trace\n");
            logLevel = sysd::LogLevel::Trace;
        }
    }
    sysd::LogInit(logLevel);

    sysd::Scheduler* scheduler = sysd::Scheduler::getInstance();

    scheduler->addPeriodicSchedulerConfig("thread_kiwi_ap", 100);
    scheduler->addPeriodicComponent(new actionPlanning::AP_Component());

    scheduler->start();

    sysd::Rate loopRate(100);
    while (scheduler->OK())
    {
        loopRate.sleep();
        printf("System D running: %s\r", sysd::toTimeString(sysd::now()).c_str()); // print system statistics
        fflush(stdout);
    }

    sysdPrintError("waitForAllThreads...\n");
    scheduler->waitForAllThreads();

    sysdPrintInfo("main return\n");
    return 0;
}