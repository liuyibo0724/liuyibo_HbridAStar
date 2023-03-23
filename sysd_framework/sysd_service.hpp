#ifndef __SERVICE_HPP_
#define __SERVICE_HPP_

#include "sysd_framework/sysd_inputPin.hpp"
#include "sysd_framework/sysd_outputPin.hpp"

#include <string>

using namespace eprosima;
using namespace std;

namespace sysd
{

template <typename SrvReqType, typename SrvRspType>
class ServiceServer
{
public:
    typedef void (*callback_t)(void *, SrvReqType &, SrvRspType &);

    ServiceServer(const char *srvName, fastdds::dds::TopicDataType *reqType, fastdds::dds::TopicDataType *rspType, DDS::DomainParticipant *participant, callback_t callback, void *data, uint32_t historySize = 0)
    {
        m_userdata = data;
        string srvNameStr = srvName;
        string topicReq = srvNameStr + "ServiceReq";
        string topicRsp = srvNameStr + "ServiceRsp";
        inputReq = new sysd::InputPin<SrvReqType>(const_cast<char *>(topicReq.c_str()), reqType, participant, 50);
        outputRsp = new sysd::OutputPin<SrvRspType>(const_cast<char *>(topicRsp.c_str()), rspType, participant);
        m_callback = callback;
    }
    ~ServiceServer()
    {
        delete inputReq;
        delete outputRsp;
    }

    bool spinOnce()
    {
        if (inputReq->readMsg(&reqMsg))
        {
            m_callback(m_userdata, reqMsg, rspMsg);
            if (outputRsp->send(rspMsg))
            {
                return true;
            }
            else
            {
                sysdPrintError("service error: server send fail\n");
                return false;
            }
        }
        else
        {
            return false;
        }
    }

private:
    void *m_userdata;
    callback_t m_callback;
    SrvReqType reqMsg;
    SrvRspType rspMsg;
    sysd::InputPin<SrvReqType> *inputReq = nullptr;
    sysd::OutputPin<SrvRspType> *outputRsp = nullptr;
};

template <typename SrvReqType, typename SrvRspType>
class ServiceClient
{
public:
    ServiceClient(const char *srvName, fastdds::dds::TopicDataType *reqType, fastdds::dds::TopicDataType *rspType, DDS::DomainParticipant *participant, uint32_t historySize = 0)
    {
        string srvNameStr = srvName;
        string topicReq = srvNameStr + "ServiceReq";
        string topicRsp = srvNameStr + "ServiceRsp";
        OutputReq = new sysd::OutputPin<SrvReqType>(const_cast<char *>(topicReq.c_str()), reqType, participant);
        InputRsp = new sysd::InputPin<SrvRspType>(const_cast<char *>(topicRsp.c_str()), rspType, participant, 50);
    }
    ~ServiceClient()
    {
        delete OutputReq;
        delete InputRsp;
    }

    //This function cannot run in PeriodicComponent because it blocks the current thread
    //timeout unit : s
    bool call(SrvReqType &req, SrvRspType &rsp, const double &timeout)
    {
        if (OutputReq->send(req))
        {
            if (InputRsp->waitForNewMsg(timeout))
            {
                InputRsp->readMsg(&rsp);
                return true;
            }
            else
            {
                sysdPrintError("service error: client receive fail\n");
                return false;
            }
        }
        else
        {
            sysdPrintError("service error: client send fail\n");
            return false;
        }
    }

private:
    SrvReqType reqMsg;
    SrvRspType rspMsg;
    sysd::OutputPin<SrvReqType> *OutputReq = nullptr;
    sysd::InputPin<SrvRspType> *InputRsp = nullptr;
};

} // namespace sysd
#endif