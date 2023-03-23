#ifndef __SYSD_COMPONENT_HPP_
#define __SYSD_COMPONENT_HPP_

#include <sys/prctl.h>
#include <functional>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

#include "sysd_framework/sysd_inputPin.hpp"
#include "sysd_framework/sysd_outputPin.hpp"

#include "sysd_framework/sysd_service.hpp"
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
using namespace eprosima;

#define UDPV4TRANSPORT 0
#define SHAREDMEMORYTRANSPORT 1

namespace sysd
{
/**
 * @brief Class Component is used to group InputPins and OutputPins into a single working unit.
 * 
 */
class Component
{
public:
    /**
     * @brief Construct a new Component object
     * 
     * @param name the name of the component
     */
    Component(const char *name = "")
    {
        int transport_method = UDPV4TRANSPORT;
      
        char *dds_transport_env = getenv("DDS_TRANSPORT_METHOD");
        transport_method  = (dds_transport_env == nullptr) ? UDPV4TRANSPORT : atoi(dds_transport_env);


        sysdPrintDebug("Component({})\n", name);
        m_name = name;
        DomainParticipantQos qos;
        //Enable Shared Memory Transport: Unable to Register SHM Transport. SHM Transport is not supported in the current platform

        if(transport_method == SHAREDMEMORYTRANSPORT){
            /**
             * @brief 使用共享内存的方式传输数据
             * 
             */
            std::shared_ptr<fastdds::rtps::SharedMemTransportDescriptor> shm_transport =
                std::make_shared<fastdds::rtps::SharedMemTransportDescriptor>();
            qos.transport().user_transports.push_back(shm_transport);
        }
        else if(transport_method == UDPV4TRANSPORT){
            /**
             * @brief 使用UDP的方式传输数据
             * 
             */
            auto udp_tranport = std::make_shared<fastdds::rtps::UDPv4TransportDescriptor>();
            // udp_tranport->sendBufferSize = 8912896;
            // udp_tranport->receiveBufferSize = 8912896;
            // udp_tranport->non_blocking_send = true;
            qos.transport().user_transports.push_back(udp_tranport);
        }
        else{
            /**
             * @brief 默认使用UDP的方式传输数据
             * 
             */
            auto udp_tranport = std::make_shared<fastdds::rtps::UDPv4TransportDescriptor>();
            // udp_tranport->sendBufferSize = 8912896;
            // udp_tranport->receiveBufferSize = 8912896;
            // udp_tranport->non_blocking_send = true;
            qos.transport().user_transports.push_back(udp_tranport);
        }

        qos.transport().use_builtin_transports = false;
        qos.name(m_name);
        uint32_t domain_id = sysd::getDomainId();
        m_participant = fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, qos);
        if (domain_id == 0)
        {
            sysdPrintWarn("{ using dds domain id 0(default), use (source ./scripts/setup.sh) to set domain id, Component: {} }\n", m_name);
        }
        sysdPrintInfo(" Component({}), using dds domain id {}", m_name, domain_id);

        if (m_participant == nullptr)
        {
            sysdPrintError("unexpected: fastdds::dds::create_participant fail");
            sysd::handleFatalError();
        }
    }
    virtual ~Component()
    {
        if (m_participant)
        {
            fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(m_participant);
        }
        sysdPrintDebug("~Component({})\n", m_name);
    }

    template <typename MsgType>
    InputPin<MsgType> *createInputPin_(const char *topic, fastdds::dds::TopicDataType *msgPubSubType, uint32_t historySize = 0)
    {
        return new sysd::InputPin<MsgType>(topic, msgPubSubType, this->m_participant, historySize);
    }
    /**
     * @brief Create a InputPin
     * 
     * @param topic Topic the InputPin will be listening
     * @param Msg message name, consistent with the idl file name
     * @param bufferSize Count of new msgs to buffer before they are read by user. Only the most recent bufferSize msgs will be kept. Set to 0 to keep all msgs.
     * @return InputPin<MsgType>* pointer to the created InputPin
     */
#define createInputPin(topic, Msg, bufferSize) createInputPin_<Msg>(topic, new Msg##PubSubType(), bufferSize)

    template <typename MsgType>
    OutputPin<MsgType> *createOutputPin_(const char *topic, fastdds::dds::TopicDataType *msgPubSubType)
    {
        return new sysd::OutputPin<MsgType>(topic, msgPubSubType, this->m_participant);
    }
    /**
     * @brief Create a OutputPin object
     * 
     * @param topic Topic the OutputPin will be writing
     * @param Msg message name, consistent with the idl file name 
     * @return OutputPin<MsgType>* pointer to the created OutputPin
     */
#define createOutputPin(topic, Msg) createOutputPin_<Msg>(topic, new Msg##PubSubType())

    template <typename MsgType>
    InputPinCache<MsgType> *createInputPinCache_(const char *topic, fastdds::dds::TopicDataType *msgPubSubType, uint32_t cacheSize = 1)
    {
        sysd::InputPin<MsgType> *inputPin = this->createInputPin_<MsgType>(topic, msgPubSubType, cacheSize); //historySize larger than cacheSize is unnecessary
        return new InputPinCache<MsgType>(inputPin, cacheSize);
    }
    /**
     * @brief Create a InputPinCache object
     * 
     * @param topic Topic the InputPin will be listening
     * @param Msg message name, consistent with the idl file name 
     * @param cacheSize the history cache size
     * @return InputPinCache<MsgType>* pointer to the created InputPinCache
     */
#define createInputPinCache(topic, Msg, cacheSize) createInputPinCache_<Msg>(topic, new Msg##PubSubType(), cacheSize)
    const char *getName()
    {
        return m_name;
    };

    template <typename SrvReqType, typename SrvRspType>
    ServiceServer<SrvReqType, SrvRspType> *createServiceServer_(const char *srvName, fastdds::dds::TopicDataType *reqType, fastdds::dds::TopicDataType *rspType, void (*callback)(void *, SrvReqType &, SrvRspType &), uint32_t historySize = 0)
    {
        return new sysd::ServiceServer<SrvReqType, SrvRspType>(srvName, reqType, rspType, this->m_participant, callback, this, historySize);
    }
#define createServiceServer(srvName, MsgReq, MsgRsp, callback) createServiceServer_<MsgReq, MsgRsp>(srvName, new MsgReq##PubSubType(), new MsgRsp##PubSubType(), callback)

    template <typename SrvReqType, typename SrvRspType>
    ServiceClient<SrvReqType, SrvRspType> *createServiceClient_(const char *srvName, fastdds::dds::TopicDataType *reqType, fastdds::dds::TopicDataType *rspType, uint32_t historySize = 0)
    {
        return new sysd::ServiceClient<SrvReqType, SrvRspType>(srvName, reqType, rspType, this->m_participant, historySize);
    }
#define createServiceClient(srvName, MsgReq, MsgRsp) createServiceClient_<MsgReq, MsgRsp>(srvName, new MsgReq##PubSubType(), new MsgRsp##PubSubType())

private:
    const char *m_name;
    fastdds::dds::DomainParticipant *m_participant = nullptr;
};

/**
 * @brief base virtual class for periodic applications to inherit
 * 
 */
class PeriodicComponent : public Component
{
public:
    /**
     * @brief Construct a new PeriodicComponent object
     * 
     * @param name the name of the component
     */
    PeriodicComponent(const char *name) : Component(name) {}

    /**
     * @brief virtual init function for subclasses to implement. 
     * if added to Scheduler by sysd::Scheduler.addPeriodicComponent(), 
     * this function will be called once after sysd::Scheduler.start();
     * 
     * @note This call should not use block codes
     * @return true 
     * @return false 
     */
    virtual bool init() = 0;
    /**
     * @brief virtual run function for subclasses to implement. 
     * if added to Scheduler by sysd::Scheduler.addPeriodicComponent, 
     * this function will be called periodically after sysd::Scheduler.start() and init();
     * 
     * @note This call should not use block codes
     * @param seq the call sequence number, starting from 0
     * @return true 
     * @return false 
     */
    virtual bool run(uint32_t seq) = 0;
};

/**
 * @brief base virtual class for thread-like applications to inherit
 * 
 */
class ThreadComponent : public Component
{
public:
    /**
     * @brief Construct a new ThreadComponent object
     * 
     * @param name the name of the component
     */
    ThreadComponent(const char *name) : Component(name) 
    {
        // set thread name
        prctl(PR_SET_NAME, name);
    }

    ~ThreadComponent()
    {
        if (m_thread != nullptr)
        {
            delete m_thread;
        }
    }

    /**
     * @brief virtual function for subclasses to implement. 
     * if added to Scheduler by sysd::Scheduler.addThreadComponent(), 
     * this function will be called once from a sperate thread, 
     * the implementation code should keep runing when sysd::Scheduler::OK() returns true, 
     * and stop(return) when sysd::Scheduler::OK() returns false.
     * 
     * @return true 
     * @return false 
     */
    virtual bool thread_main() = 0;

    bool start()
    {
        m_thread = new std::thread(&ThreadComponent::thread_main, this);
        if (m_thread == nullptr)
        {
            sysdPrintError("unexpected: new std::thread fail");
            sysd::handleFatalError();
        } 
        // else {
        //     pthread_setname_np(m_thread->native_handle() , "sysd_component");
        // }
        return true;
    }
    void join()
    {
        m_thread->join();
    }

private:
    std::thread *m_thread;
};

} // namespace sysd

#endif
