#ifndef ED_TIME_H_
#define ED_TIME_H_

#include <iostream>

namespace ed
{

class Time
{

public:
    Time() : secs_(0) {}

    Time(double secs) : secs_(secs) {}

    bool operator<(const Time& rhs) const { return secs_ < rhs.secs_; }

    friend std::ostream& operator<<(std::ostream& out, const Time& d)
    {
        int const isecs = static_cast<int>(d.secs_);

        int const h = isecs / 3600;
        int const m = (isecs / 60) % 60;
        int const s = isecs % 60;
        int const ms = static_cast<int>(1000 * (d.secs_ - isecs));

        out << h << ":" << m << ":" << s << ":" << ms;

        return out;
    }

    [[nodiscard]]
    double seconds() const
    {
        return secs_;
    }

private:
    double secs_;
};

} // end namespace ed

#endif
