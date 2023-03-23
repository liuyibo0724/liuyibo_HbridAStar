#ifndef _SYSD_DIAGNOSIS_HPP_
#define _SYSD_DIAGNOSIS_HPP_

#include "sysd_framework/sysd_scheduler.hpp"

#include "idl_generated/DiagnosisPubSubTypes.h"

using namespace std;

namespace sysd
{
inline void setErrorcode(uint32_t newcode, uint32_t &code)
{
    code = (code == 0)?newcode:code;
}

inline void sendErrorcode(sysd::ServiceClient<DiagnosisReq, DiagnosisRsp> *srvClient,uint32_t &errorcode)
{
    while (sysd::Scheduler::OK())
    {
        DiagnosisReq reqData;
        DiagnosisRsp rspData;
        reqData.errorcode = errorcode;
        rspData.iserrorexit = false;
        if (errorcode != 0)
        {
            if (srvClient->call(reqData, rspData, 3.0))
            {
                // sysdPrintDebug("call dg m_errorcode:{}\n", errorcode);
            }
            else
            {
                sysdPrintError("DG serviceClient response error\n");
            }
        }
        //如果在初始化阶段或构造函数那有错误码(配置错误)则保留错误码，否则清零
        if (((errorcode / 1000) % 10) != 2)
        {
            errorcode = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}
} // namespace sysd
#endif