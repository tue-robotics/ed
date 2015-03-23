#ifndef ERA_TUE_CONFIGURATION_VARIANT_H_
#define ERA_TUE_CONFIGURATION_VARIANT_H_

#include <vector>
#include <map>
#include <string>
#include <iostream>

namespace ed
{

namespace io
{

class Variant
{

public:

    Variant() : type_('?') {}

    Variant(const double& d) : type_('d'), d_(d) {}
    Variant(int i) : type_('i'), i_(i) {}
    Variant(const std::string& s) : type_('s'), s_(s) {}
    Variant(const char* s) : type_('s'), s_(s) {}

    bool getValue(int& v) const { return checkAndGet(i_, 'i', v); }
    bool getValue(double& v) const { return checkAndGet(d_, 'd', v) || checkAndGet((double)i_, 'i', v); }
    bool getValue(float& v) const { return checkAndGet((float)d_, 'd', v) || checkAndGet((float)i_, 'i', v); }
    bool getValue(std::string& v) const { return checkAndGet(s_, 's', v); }

    bool getValue(bool& v) const
    {
        int i;
        if (!checkAndGet(i_, 'i', i))
            return false;
        v = (i == 1);
        return true;
    }

    bool isString() const { return type_ == 's'; }

    bool inline valid() const { return type_ != '?'; }

private:

    char type_;

    union {
        int i_;
        double d_;
    };

    std::string s_;

    template<typename T>
    inline bool checkAndGet(const T& v, char type, T& out) const
    {
        if (type != type_)
            return false;
        out = v;
        return true;
    }

    friend std::ostream& operator<< (std::ostream& out, const Variant& v)
    {
        switch (v.type_)
        {
        case 'i': out << v.i_;
            break;
        case 'd': out << v.d_;
            break;
        case 's': out << v.s_;
            break;
        default: out << "?";
            break;
        }

        return out;
    }

};

}

}

#endif
