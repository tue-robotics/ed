#ifndef ED_UUID_H_
#define ED_UUID_H_

#include "ed/types.h"
#include <string>
#include <utility>

namespace ed
{

class UUID
{

public:
    UUID() : idx(INVALID_IDX) {}
    UUID(const char* s) : id_(s), idx(INVALID_IDX) {}
    UUID(std::string s) : id_(std::move(s)), idx(INVALID_IDX) {}

    bool operator<(const UUID& rhs) const { return id_ < rhs.id_; }

    bool operator==(const UUID& rhs) const { return id_ == rhs.id_; }

    bool operator!=(const UUID& rhs) const { return id_ != rhs.id_; }

    const char* c_str() const { return id_.c_str(); }

    const std::string& str() const { return id_; }

    friend std::ostream& operator<<(std::ostream& out, const UUID& d)
    {
        out << d.id_;
        return out;
    }

private:
    std::string id_;

public:
    mutable Idx idx;
};

} // end namespace ed

#endif
