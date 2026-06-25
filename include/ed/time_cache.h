#ifndef ED_TIME_CACHE_H_
#define ED_TIME_CACHE_H_

#include "ed/time.h"
#include <map>

namespace ed
{

template <typename T> class TimeCache
{

public:
    using const_iterator = typename std::map<Time, T>::const_iterator;

    TimeCache() = default;

    ~TimeCache() = default;

    void insert(const Time& t, const T& value)
    {
        if (max_size_ > 0 && cache_.size() == max_size_)
            cache_.erase(cache_.begin());
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

    [[nodiscard]]
    const_iterator begin() const
    {
        return cache_.begin();
    }
    [[nodiscard]]
    const_iterator end() const
    {
        return cache_.end();
    }

    [[nodiscard]]
    unsigned int size() const
    {
        return cache_.size();
    }

    void setMaxSize(unsigned int n) { max_size_ = n; }

private:
    // Cache with items ordered in time
    std::map<Time, T> cache_;

    unsigned int max_size_{0};
};

} // end namespace ed

#endif
