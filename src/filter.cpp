#include "filter.h"

using namespace HybridAStar;

template<class T>
void signalsFilter<T>::setSignals(T T_signal)
{
    m_signals.push_front(T_signal);
    m_signals.pop_back();
}

template<class T>
void signalsFilter<T>::setAllSignals(T T_signal)
{
    for(int i = 0; i < m_signals.size(); i ++) m_signals[i] = T_signal;
}

template<class T>
bool signalsFilter<T>::aveFilter()
{
    if(m_signals.empty()) return false;
    else
    {
        T result = (m_signals[0] * 15.f + m_signals[1] * 10.f + m_signals[2] * 6.f + m_signals[3] * 3.f +m_signals[4] * 1.f) / 35.f;
        this->m_filteredSig = result;
        return true;
    }

}

template<class T>
T signalsFilter<T>::run(T T_signal)
{
    setSignals(T_signal);
    aveFilter();
    return getfilteredSig();
}