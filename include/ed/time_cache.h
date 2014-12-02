#ifndef ED_TIME_CACHE_H_
#define ED_TIME_CACHE_H_

#include "ed/time.h"
#include <map>

namespace ed
{

template<typename T>
class TimeCache
{

public:

    typedef typename std::map<Time, T>::const_iterator const_iterator;

    TimeCache() {}

    ~TimeCache() {}

    void insert(const Time& t, const T& value)
    {
        cache_[t] = value;
    }

    void getLowerUpper(const Time& t, const_iterator& lower, const_iterator& upper) const
    {
        if (cache_.empty())
        {
            lower = cache_.end();
            upper = cache_.end();
            return;
        }

        // std::map::upper_bound returns first iterator after key
        upper = cache_.upper_bound(t);

        if (upper == cache_.begin())
        {
            lower = cache_.end();
        }
        else
        {
            lower = upper;
            --lower;
        }
    }

    inline const_iterator begin() const { return cache_.begin(); }
    inline const_iterator end() const { return cache_.end(); }

private:

    // Cache with items ordered in time
    std::map<Time, T> cache_;

};

} // end namespace ed

#endif
