#ifndef __SYSD_OUTPUTPIN_HPP_
#define __SYSD_OUTPUTPIN_HPP_

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>

#include "sysd_framework/common.hpp"

namespace DDS = eprosima::fastdds::dds;

namespace sysd
{

/**
 * @brief class for writing messages to dds bus
 * 
 * @tparam MsgType message type
 */
template <typename MsgType>
class OutputPin
{
public:
    /**
     * @brief Construct a new OutputPin object
     * 
     * @param topic Topic the OutputPin will be writing to
     * @param msgPubSubType PubSubType of the message type
     * @param participant belonging dds participant of the OutputPin
     */
    OutputPin(const char *topic, DDS::TopicDataType *msgPubSubType, DDS::DomainParticipant *participant)
        : m_MsgPubSubType(msgPubSubType), m_belongingParticipant(participant)
    {
        sysdPrintDebug("OutputPin({}, {}(default size={}))\n",
               topic, m_MsgPubSubType.get_type_name().c_str(), sizeof(MsgType));

        //register msg type to participant
        m_MsgPubSubType.register_type(m_belongingParticipant); //todo segmentation fault when destructing?

        // Create the publications Topic
        m_topic = m_belongingParticipant->create_topic(topic, m_MsgPubSubType.get_type_name(), DDS::TOPIC_QOS_DEFAULT);
        if (m_topic == nullptr)
        {
            sysdPrintError("create_topic fail, maybe multi topics in the same component");
            sysd::handleFatalError();
        }

        // Create the Publisher
        m_publisher = m_belongingParticipant->create_publisher(DDS::PUBLISHER_QOS_DEFAULT);
        if (m_publisher == nullptr)
        {
            sysdPrintError("unexpected: create_publisher fail");
            sysd::handleFatalError();
        }
        // Create the DataWriter with topic
        m_writer = m_publisher->create_datawriter(m_topic, DDS::DATAWRITER_QOS_DEFAULT, nullptr);
        if (m_writer == nullptr)
        {
            sysdPrintError("unexpected: create_datawriter fail");
            sysd::handleFatalError();
        }
    }
    ~OutputPin()
    {
        sysdPrintDebug("~OutputPin({})\n", m_topic->get_name().c_str());
        if (m_writer)
        {
            m_publisher->delete_datawriter(m_writer);
        }
        if (m_publisher)
        {
            m_belongingParticipant->delete_publisher(m_publisher);
        }
        if (m_topic)
        {
            m_belongingParticipant->delete_topic(m_topic);
        }
    }
    /**
     * @brief Write data to the topic.
     * 
     * @param data reference of the data
     * @return bool 
     */
    bool send(MsgType &data)
    {
        m_writer->write(&data); //may block
        sysdPrintDebug("\t\tOutputPin({}:{}).send: msg size={}, cdr size={}",  m_belongingParticipant->get_qos().name().c_str(),
               m_topic->get_name().c_str(),
            //    data.seq,
               sizeof(MsgType),
               MsgType::getCdrSerializedSize(data));
        return true;
    }

private:
    DDS::Publisher *m_publisher = nullptr;
    DDS::DataWriter *m_writer = nullptr;
    DDS::Topic *m_topic = nullptr;

    DDS::TypeSupport m_MsgPubSubType;

    DDS::DomainParticipant *m_belongingParticipant = nullptr;
};
} // namespace sysd
#endif