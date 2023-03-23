#ifndef __SYSD_PINTEMPLATES_HPP_
#define __SYSD_PINTEMPLATES_HPP_

#include "sysd_framework/common.hpp"

template <typename MsgPtrType, typename PinPtrType>
inline bool sysdGetNewestSignalTemplate(MsgPtrType &msg, PinPtrType pin, sysd::Time &timeout, sysd::Time *oldestTimeStamp)
{
    sysd::Time msgTime;

    msg = pin->getNewestMsgSince(sysd::now() - timeout, &msgTime);
    if (nullptr == msg)
    {
        return false;
    }
    if (oldestTimeStamp && (msgTime < *oldestTimeStamp))
    {
        *oldestTimeStamp = msgTime;
    }
    return true;
}

#endif