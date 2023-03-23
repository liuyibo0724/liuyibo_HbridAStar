#ifndef __SYSD_INPUTPIN_HPP_
#define __SYSD_INPUTPIN_HPP_

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <boost/circular_buffer.hpp>

namespace DDS = eprosima::fastdds::dds;

namespace sysd
{
/**
 * @brief class for reading messages from dds bus
 * 
 * @tparam MsgType message type
 */
template <typename MsgType>
class InputPin
{
public:
    /**
     * @brief Construct a new InputPin object
     * 
     * @param topic Topic the InputPin will be listening
     * @param msgPubSubType PubSubType of the message type
     * @param participant belonging dds participant of the InputPin
     * @param historySize controls the dds should deliver only the most recent-historySize value; if 0, dds should attempt to deliver all intermediate values
     */
    InputPin(const char *topic, DDS::TopicDataType *msgPubSubType, DDS::DomainParticipant *participant, uint32_t historySize = 0)
        : m_MsgPubSubType(msgPubSubType), m_belongingParticipant(participant)
    {
        sysdPrintDebug("InputPin({}, {})\n", topic, m_MsgPubSubType.get_type_name().c_str());

        //register msg type to participant
        m_MsgPubSubType.register_type(m_belongingParticipant);

        // Create the subscriptions Topic
        m_topic = m_belongingParticipant->create_topic(topic, m_MsgPubSubType.get_type_name(), DDS::TOPIC_QOS_DEFAULT);
        if (m_topic == nullptr)
        {
            sysdPrintError("create_topic fail, maybe multi topics in the same component");
            sysd::handleFatalError();
        }

        // Create the Subscriber
        m_subscriber = m_belongingParticipant->create_subscriber(DDS::SUBSCRIBER_QOS_DEFAULT);
        if (m_subscriber == nullptr)
        {
            sysdPrintError("unexpected: create_subscriber fail");
            sysd::handleFatalError();
        }

        DDS::DataReaderQos qos;
        if (historySize == 0)
        {
            qos.history().kind = DDS::KEEP_ALL_HISTORY_QOS;
        }
        else
        {
            qos.history().kind = DDS::KEEP_LAST_HISTORY_QOS;
            qos.history().depth = historySize;
        }

        m_reader = m_subscriber->create_datareader(m_topic, qos);
        if (m_reader == nullptr)
        {
            sysdPrintError("unexpected: create_datareader fail");
            sysd::handleFatalError();
        }
    }
    ~InputPin()
    {
        sysdPrintDebug("~InputPin({})\n", m_topic->get_name().c_str());
        if (m_reader != nullptr)
        {
            m_subscriber->delete_datareader(m_reader);
        }
        if (m_subscriber != nullptr)
        {
            m_belongingParticipant->delete_subscriber(m_subscriber);
        }
        if (m_topic != nullptr)
        {
            m_belongingParticipant->delete_topic(m_topic);
        }
    }
    /**
     * @brief Method to block the current thread until a new message is available
     * 
     * @param timeout Max blocking time for this operation
     * @return true true if there is new unread message
     * @return false false if timeout
     */
    bool waitForNewMsg(double timeout)
    {
        eprosima::fastrtps::Duration_t duration(timeout);
        return m_reader->wait_for_unread_message(duration);
    }

    /**
     * @brief This operation copies the next Data value from the DataReader and ‘removes’ it from the DataReader.
     * 
     * @param msg Message pointer to store the message
     * @param timestamp Time pointer to store the source_timestamp, nullptr means don't care
     * @return true 
     * @return false 
     */
    bool readMsg(MsgType *msg, sysd::Time *timestamp = nullptr)
    {
        if (msg == nullptr)
        {
            sysdPrintError("readMsg: parameter msg = nullptr\n");
            handleFatalError();
        }

        DDS::SampleInfo info;
        if (m_reader->take_next_sample(msg, &info) == ReturnCode_t::RETCODE_OK)
        {
            if (timestamp)
                *timestamp = info.source_timestamp;
            sysdPrintDebug("\t\t\tInputPin({}:{}).recv: @{}\n", m_belongingParticipant->get_qos().name().c_str(),
                           m_topic->get_name().c_str(),
                           //    msg->seq,
                           sysd::toTimeString(info.source_timestamp).c_str());
            return true;
        }
        else
        {
            return false;
        }
    }
    using MsgCallbackType = void (*)(MsgType *msg, sysd::Time *timestamp);
    void registerMsgCallback(MsgCallbackType callback)
    {
        m_msgCallback = callback;
    }
    void spin()
    {
        MsgType msg;
        sysd::Time timestamp;
        if (readMsg(&msg, &timestamp))
        {
            m_msgCallback(&msg, &timestamp);
        }
    }
    DDS::Topic *getTopic()
    {
        return m_topic;
    }
    MsgType msg;
        sysd::Time srcTime;
    bool readNewestSample(MsgType* msg,sysd::Time *srcTime)
    {
        DDS::SampleInfo info;
        if(m_reader->read_newest_sample(msg, &info) == ReturnCode_t::RETCODE_OK )
        {
            *srcTime = info.source_timestamp;
            return true;
        }
        return false;
    }
    bool readSyncedSample(MsgType* msg,sysd::Time *srcTime,sysd::Time synced_time,sysd::Time delta)
    {
        DDS::SampleInfo info;
        if(m_reader->read_synced_sample(msg, &info, synced_time, delta) == ReturnCode_t::RETCODE_OK )
        {
            *srcTime = info.source_timestamp;
            return true;
        }
        return false;
    }
private:
    DDS::Subscriber *m_subscriber = nullptr;
    DDS::DataReader *m_reader = nullptr;
    DDS::Topic *m_topic = nullptr;

    DDS::TypeSupport m_MsgPubSubType;

    DDS::DomainParticipant *m_belongingParticipant = nullptr;
    MsgCallbackType m_msgCallback;
};

/**
 * @brief Class to buffer InputPin's msg to history cache, then provides api for getting newest message/ most synchronized message
 * 
 * @tparam MsgType message type
 */
template <typename MsgType>
class InputPinCache
{
public:
    /**
     * @brief Construct a new InputPinCache object
     * 
     * @param pin InputPin pointer to buffer
     * @param cacheSize cache size of history
     */
    InputPinCache(InputPin<MsgType> *pin, uint32_t cacheSize = 10) : m_pin(pin)
    {
        m_cache = new boost::circular_buffer<MsgAndInfo>(cacheSize);
        if (m_cache == nullptr)
        {
            sysdPrintError("unexpected: new boost::circular_buffer fail");
            sysd::handleFatalError();
        }
    }
    ~InputPinCache()
    {
        if (m_cache != nullptr)
        {
            delete m_cache;
        }
        if (m_pin != nullptr)
        {
            delete m_pin;
        }
    }

    /**
     * @brief Get the newest received message pointer
     * 
     * @param srcTime pointer to msg source_timestamp, nullptr means don't care
     * @return const MsgType* const pointer to the newest received message, or nullptr if there is no history yet
     */
    // const MsgType *getNewestMsg(sysd::Time *srcTime = nullptr)
    // {
    //     this->rxUpdate();

    //     if (m_cache->empty())
    //     {
    //         return nullptr;
    //     }
    //     const MsgAndInfo &msgAndInfo = m_cache->back();
    //     if (srcTime)
    //         *srcTime = msgAndInfo.srcTime;
    //     return &(msgAndInfo.msg);
    // }
    const MsgType *getNewestMsg(sysd::Time *srcTime = nullptr)
    {
        if(this->m_pin->readNewestSample(&m_msgAndInfo.msg, &m_msgAndInfo.srcTime))
        {
            if (srcTime)
            {
                *srcTime = m_msgAndInfo.srcTime;
            }
                
            return &(m_msgAndInfo.msg);
        }
        return nullptr;
    }
    /**
     * @brief Get the newest received message pointer
     * 
     * @param srcTime pointer to msg source_timestamp, nullptr means don't care
     * @return const MsgType* const pointer to the newest received message, or nullptr if there is no history yet
     */
    const MsgType *getNewestMsgSince(sysd::Time sinceTime, sysd::Time *srcTime = nullptr)
    {
        sysd::Time srcTime_local;
        const MsgType *msg = getNewestMsg(&srcTime_local);
        if (nullptr == msg)
        {
            return nullptr;
        }
        if (srcTime_local >= sinceTime)
        {
            if (srcTime)
                *srcTime = srcTime_local;
            return msg;
        }
        else
        {
            sysdPrintWarn("{} getNewestMsgSince fail: sinceTime={}, nearest={}\n",
                          this->m_pin->getTopic()->get_name().c_str(),
                          toTimeString(sinceTime).c_str(),
                          toTimeString(srcTime_local).c_str());
            return nullptr;
        }
    }

    /**
     * @brief get the synchronized message pointer, whose timestamp
     * is in range [timestamp - delta, timestamp + delta] and closest to <timestamp>.
     * 
     * @param timestamp 
     * @param delta 
     * @param srcTime pointer to msg source_timestamp, nullptr means don't care
     * @return const MsgType* The message pointer, or nullptr if requirements are not satisfied.
     */
    // const MsgType *getSyncedMsg(const sysd::Time &timestamp, const sysd::Time &delta,
    //                             sysd::Time *srcTime = nullptr)
    // {
    //     this->rxUpdate();
    //     const MsgAndInfo *nearest = nullptr;
    //     for (auto item = m_cache->begin(); item != m_cache->end(); ++item)
    //     {
    //         if (nearest == nullptr || absTimeDiff(item->srcTime, timestamp) < absTimeDiff(nearest->srcTime, timestamp))
    //         {
    //             nearest = &(*item);
    //         }
    //     }
    //     if (nearest != nullptr && absTimeDiff(nearest->srcTime, timestamp) <= delta)
    //     {
    //         if (srcTime)
    //             *srcTime = nearest->srcTime;
    //         return &(nearest->msg);
    //     }
    //     else
    //     {
    //         sysdPrintWarn("{} getSyncedMsg fail: timestamp={}, delta={:f}ms, nearest={}\n", this->m_pin->getTopic()->get_name().c_str(), toTimeString(timestamp).c_str(), delta.to_ns() / 1000000.0,
    //                       nearest == nullptr ? "null" : toTimeString(nearest->srcTime).c_str());
    //         return nullptr;
    //     }
    // }
    const MsgType *getSyncedMsg(const sysd::Time &timestamp, const sysd::Time &delta,
                                sysd::Time *srcTime = nullptr)
    {
        if(this->m_pin->readSyncedSample(&m_msgAndInfoSynced.msg, &m_msgAndInfoSynced.srcTime,timestamp,delta))
        {
            if (srcTime)
                *srcTime = m_msgAndInfoSynced.srcTime;
            return &(m_msgAndInfoSynced.msg);
        }
        else
        {
            return nullptr;
        }
    }
private:
    InputPin<MsgType> *m_pin;
    struct MsgAndInfo
    {
        MsgType msg;
        sysd::Time srcTime;
    };
    boost::circular_buffer<MsgAndInfo> *m_cache;
    
    MsgAndInfo m_msgAndInfo;
    MsgAndInfo m_msgAndInfoSynced;
    uint32_t rxUpdate()
    {
        MsgAndInfo msgAndInfo;
        uint32_t count = 0;
        while (m_pin->readMsg(&msgAndInfo.msg, &msgAndInfo.srcTime))
        {
            m_cache->push_back(msgAndInfo);
            count++;
        }
        return count;
    }
    /// @brief Private fuction to caculate absolute difference of
    /// two Time value.
    Time absTimeDiff(const Time &a, const Time &b)
    {
        if (a > b)
            return a - b;
        else
            return b - a;
    }
};

} // namespace sysd
#endif
