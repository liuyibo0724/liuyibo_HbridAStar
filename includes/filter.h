#ifndef FILTER_H
#define FILTER_H

#include <queue>

namespace HybridAStar
{
    //各种滤波器
    template<class T>
    class signalsFilter
    {
    public:
        void setSignals();                                                      //从接口中读取信号队列
        T getfilteredSig() const { return m_filteredSig; }   //读取待处理信号队列
        bool aveFilter();                                                          //均值过滤核心函数
    private:
        std::queue<T, std::deque<T>> m_signals;                                 //待处理的信号队列
        T m_filteredSig;                                                        //过滤过的当前信号
    };
}

#endif  //FILTER_H