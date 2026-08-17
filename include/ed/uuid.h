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
    // A UUID is a transparent string wrapper; ed::UUID id = "foo" is the intended spelling.
    // NOLINTBEGIN(google-explicit-constructor)
    UUID(const char* s) : id_(s), idx(INVALID_IDX) {}
    UUID(std::string s) : id_(std::move(s)), idx(INVALID_IDX) {}
    // NOLINTEND(google-explicit-constructor)

    bool operator<(const UUID& rhs) const { return id_ < rhs.id_; }

    bool operator==(const UUID& rhs) const { return id_ == rhs.id_; }

    bool operator!=(const UUID& rhs) const { return id_ != rhs.id_; }

    // Mirrors std::string::c_str().
    // NOLINTNEXTLINE(readability-identifier-naming)
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
