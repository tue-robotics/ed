#ifndef ED_PROPERTY_KEY_DB_H_
#define ED_PROPERTY_KEY_DB_H_

#include "ed/property_info.h"
#include "ed/property_key.h"
#include "ed/types.h"

#include <map>

namespace ed
{

struct PropertyKeyDBEntry
{
    PropertyKeyDBEntry() = default;

    ~PropertyKeyDBEntry() { delete info; }

    std::string name;
    PropertyInfo* info{nullptr};
    Idx idx{};
};

class PropertyKeyDB
{

public:
    ~PropertyKeyDB()
    {
        for (auto& it : name_to_info_)
        {

            delete it.second;
        }
    }

    template <typename T>
    void registerProperty(const std::string& name, PropertyKey<T>& key, PropertyInfo* info = nullptr)
    {
        PropertyKeyDBEntry* entry = nullptr;

        auto const it = name_to_info_.find(name);
        if (it == name_to_info_.end())
        {
            entry = new PropertyKeyDBEntry;
            entry->name = name;
            entry->idx = name_to_info_.size();

            if (info)
                entry->info = info;
            else
                entry->info = new PropertyInfo;

            name_to_info_[name] = entry;
        }
        else
        {
            entry = it->second;

            if (info)
            {
                // TODO: needs locking? (Keys may access the entry info at this point)
                delete entry->info;
                entry->info = info;
            }
        }

        key.entry = entry;
        key.idx = entry->idx;
    }

    [[nodiscard]]
    const PropertyKeyDBEntry* getPropertyKeyDBEntry(const std::string& name) const
    {
        auto const it = name_to_info_.find(name);
        if (it == name_to_info_.end())
            return nullptr;

        return it->second;
    }

private:
    std::map<std::string, PropertyKeyDBEntry*> name_to_info_;
};

} // namespace ed

#endif
