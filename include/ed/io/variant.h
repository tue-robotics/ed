#ifndef ERA_TUE_CONFIGURATION_VARIANT_H_
#define ERA_TUE_CONFIGURATION_VARIANT_H_

#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ed::io
{

class Variant
{

public:
    Variant() : type_('?') {}

    // A variant exists to be constructed from any of its alternatives; implicit is the point.
    // NOLINTBEGIN(google-explicit-constructor)
    Variant(const double& d) : type_('d'), d_(d) {}
    Variant(int i) : type_('i'), i_(i) {}
    Variant(std::string s) : type_('s'), s_(std::move(s)) {}
    Variant(const char* s) : type_('s'), s_(s) {}
    // NOLINTEND(google-explicit-constructor)

    bool getValue(int& v) const { return checkAndGet(i_, 'i', v); }
    bool getValue(double& v) const { return checkAndGet(d_, 'd', v) || checkAndGet(static_cast<double>(i_), 'i', v); }
    bool getValue(float& v) const
    {
        return checkAndGet(static_cast<float>(d_), 'd', v) || checkAndGet(static_cast<float>(i_), 'i', v);
    }
    bool getValue(std::string& v) const { return checkAndGet(s_, 's', v); }

    bool getValue(bool& v) const
    {
        int i = 0;
        if (!checkAndGet(i_, 'i', i))
            return false;
        v = (i == 1);
        return true;
    }

    [[nodiscard]]
    bool isString() const
    {
        return type_ == 's';
    }

    [[nodiscard]]
    bool valid() const
    {
        return type_ != '?';
    }

private:
    char type_;

    int i_{};
    double d_{};

    std::string s_;

    template <typename T> bool checkAndGet(const T& v, char type, T& out) const
    {
        if (type != type_)
            return false;
        out = v;
        return true;
    }

    friend std::ostream& operator<<(std::ostream& out, const Variant& v)
    {
        switch (v.type_)
        {
        case 'i': out << v.i_; break;
        case 'd': out << v.d_; break;
        case 's': out << v.s_; break;
        default: out << "?"; break;
        }

        return out;
    }
};

} // namespace ed::io

#endif
