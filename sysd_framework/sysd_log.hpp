#ifndef __SYSD_LOG_HPP_
#define __SYSD_LOG_HPP_

#include <ctime>
#include <chrono>

#include <fastdds/dds/log/Log.hpp>
#include <fastdds/dds/log/FileConsumer.hpp>
#include <stdarg.h>
#include <sys/stat.h>
#include <dirent.h>
#include <malloc.h>
#include <stdlib.h>


// using spdlog 
//
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE  // open debug and trace log
#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <iostream>


#define SYSD_VERBOSE_LEVEL_INFO 2
#define SYSD_VERBOSE_LEVEL_WARN 1
#define SYSD_VERBOSE_LEVEL_ERROR 0
#define SYSD_VERBOSE_LEVEL_DEBUG 3
#define SYSD_VERBOSE_LEVEL_TRACE 4

#define sysdStaticVerboseLevel SYSD_VERBOSE_LEVEL_ERROR

#define DROP_LOG_FILE 0
#define SAVE_LOG_FILE 1

#define sysdLogFileSavedOption SAVE_LOG_FILE

namespace sysd
{

#define sysdPrintDebug(...) SPDLOG_LOGGER_DEBUG(spdlog::get("idc"), __VA_ARGS__)
#define sysdPrintInfo(...) SPDLOG_LOGGER_INFO(spdlog::get("idc"), __VA_ARGS__)
#define sysdPrint(...) SPDLOG_LOGGER_INFO(spdlog::get("idc"), __VA_ARGS__)
#define sysdPrintWarn(...) SPDLOG_LOGGER_WARN(spdlog::get("idc"), __VA_ARGS__)
#define sysdPrintError(...) SPDLOG_LOGGER_ERROR(spdlog::get("idc"), __VA_ARGS__)

//#define sysdPrintDebug(...) SPDLOG_LOGGER_DEBUG(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_DEBUG(spdlog::get("idc"), __VA_ARGS__)
//#define sysdPrintInfo(...) SPDLOG_LOGGER_INFO(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_INFO(spdlog::get("idc"), __VA_ARGS__)
//#define sysdPrint(...) SPDLOG_LOGGER_INFO(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_INFO(spdlog::get("idc"), __VA_ARGS__)
//#define sysdPrintWarn(...) SPDLOG_LOGGER_WARN(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_WARN(spdlog::get("idc"), __VA_ARGS__)
//#define sysdPrintError(...) SPDLOG_LOGGER_ERROR(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_ERROR(spdlog::get("idc"), __VA_ARGS__)
    
//#define sysdPrintDebug(format, args...) 
//#define sysdPrintInfo(format, args...)  
//#define sysdPrint(format, args...)      
//#define sysdPrintWarn(format, args...)  
//#define sysdPrintError(format, args...) 


// *************** the following code is no longer need*************//

/**************************************** dds log ******************************
#if sysdStaticVerboseLevel >= SYSD_VERBOSE_LEVEL_INFO
#define sysdPrint(format, args...) sysd::sysdPrintf(eprosima::fastdds::dds::Log::Info, format, ##args)
#define sysdPrintInfo(format, args...) sysd::sysdPrintf(eprosima::fastdds::dds::Log::Info, format, ##args)
#else 
#define sysdPrint(format, args...) 
#define sysdPrintInfo(format, args...) 
#endif
#if sysdStaticVerboseLevel >= SYSD_VERBOSE_LEVEL_WARN
#define sysdPrintWarn(format, args...) sysd::sysdPrintf(eprosima::fastdds::dds::Log::Warning, format, ##args)
#else 
#define sysdPrintWarn(format, args...) 
#endif
#if sysdStaticVerboseLevel >= SYSD_VERBOSE_LEVEL_ERROR
#define sysdPrintError(format, args...) sysd::sysdPrintf(eprosima::fastdds::dds::Log::Error, format"\n\t\t\t\t@ {}:{:d}", ##args, __FILE__, __LINE__)
#else 
#define sysdPrintError(format, args...)
#endif

#if sysdStaticVerboseLevel >= SYSD_VERBOSE_LEVEL_DEBUG
#define sysdPrintDebug(format, args...) sysd::sysdPrintf(eprosima::fastdds::dds::Log::Info, format, ##args)
#else 
#define sysdPrintDebug(format, args...) 
#endif

**************************************** dds log end******************************/
using namespace eprosima::fastdds::dds;

enum LogLevel
{
    Error,
    Warn,
    Info,
    Debug,
    Trace,
};

//extern sysd::LogLevel g_logLevel;

/**
 * @brief config of fastdds log lib
 * 
 */
inline void LogInit(LogLevel logLevel)
{
    if (sysdLogFileSavedOption) //save log to file
    {
        char* sysd_config_path = getenv("HOME");

        if (sysd_config_path == nullptr)
        {
            sysd_config_path = (char*)".";
        }
        std::string sysd_config_path_ = std::string(sysd_config_path) + "/logs";
        // sysd_config_path = (char*)sysd_config_path_.c_str();
        // strcpy(sysd_config_path,sysd_config_path_.c_str());

        if(access(sysd_config_path_.c_str(), 0) == -1){
            sysd_config_path_ = "/map/logs";
            if(access(sysd_config_path_.c_str(), 0) == -1){
                std::string make_logdir = "mkdir -p /map/logs" ;
                system(make_logdir.c_str());
            }
        }
        #ifdef J3_COMPONENTS_BUILD
            sysd_config_path_ = "/map/logs/dmk";
        #endif
        
        char timeBuffer[100];
        time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        struct tm *tm = localtime(&now);
        strftime(timeBuffer, 100, "%Y%m%d_%H-%M-%S", tm);

        char dateBuffer[100];
        strftime(dateBuffer, 100, "%Y%m%d", tm);
        // std::string make_logfile = "mkdir " + std::string(dateBuffer);
        // system(make_logfile.c_str());

        //std::string idc_log_filename  = sysd_config_path_ + "/logs/log_idc" + std::string(timeBuffer) + ".log";
        // std::string idc_log_filename = sysd_config_path_ + "/logs/log_idc.log";

        // #ifdef J3_COMPONENTS_BUILD
        // std::string idc_log_filename = "/map/logs/log_idc.log";
        // #else
        std::string idc_log_filename = sysd_config_path_ + "/log_idc.log";
        // #endif
        printf("The logs file record path : %s \n", idc_log_filename.c_str());

        auto logger = spdlog::rotating_logger_mt("idc", idc_log_filename, 50 * 1024 * 1024, 20); //50M
        logger->flush_on(spdlog::level::err);  
        spdlog::flush_every(std::chrono::seconds(1)); //flush date to file every 3s 
        auto console = spdlog::stdout_color_mt("console");
        spdlog::set_default_logger(console);
        spdlog::set_pattern("%^%Y-%m-%d %H:%M:%S.%e [%l] [thread %t] - <%s>:<%#>|<%!>,%$ %v");

        if(logLevel == LogLevel::Error){
            spdlog::set_level(spdlog::level::err); // Set global log level 
        }else if(logLevel == LogLevel::Warn){
            spdlog::set_level(spdlog::level::warn); 
        }else if(logLevel == LogLevel::Info){
            spdlog::set_level(spdlog::level::info); 
        }else if(logLevel == LogLevel::Debug){
            spdlog::set_level(spdlog::level::debug); 
        }else if(logLevel == LogLevel::Trace)
            spdlog::set_level(spdlog::level::trace); 
        else 
            spdlog::set_level(spdlog::level::warn); 


/*
 *        struct dirent **namelist;
 *        int n = scandir(sysd_config_path, &namelist, nullptr,alphasort);
 *        if(n > 0)
 *        {
 *            for(size_t count=0; n-- > 0;)
 *            {
 *                if (strstr(namelist[n]->d_name, "logidc_"))
 *                {
 *                    if(++count > 4)
 *                    {
 *                        std::string shell_rm_logfile = "rm -f " + std::string(sysd_config_path) + "/" + std::string(namelist[n]->d_name);
 *                        system(shell_rm_logfile.c_str());
 *                    }
 *                }
 *                free(namelist[n]);
 *            }
 *            free(namelist);
 *        }
 *
 */
    }
}


} // namespace sysd
#endif
