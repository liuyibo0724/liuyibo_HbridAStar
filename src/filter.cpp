#include "filter.h"

using namespace HybridAStar;

template<class T>
bool signalsFilter<T>::aveFilter()
{
    if(m_signals.empty()) return false;
    else
    {
        T result = 0;
        for(T T_tmp : m_signals) result += T_tmp;
        result /= m_signals.size();
        this->m_filteredSig = result;
        return true;
    }

}