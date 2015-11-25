#ifndef ED_PROPERTY_KEY_DB_H_
#define ED_PROPERTY_KEY_DB_H_

#include "ed/types.h"
#include "ed/property_key.h"
#include "ed/property_info.h"

#include <map>

namespace ed
{

struct PropertyKeyDBEntry
{
    PropertyKeyDBEntry() : info(0) {}

    ~PropertyKeyDBEntry()
    {
        delete info;
    }

    std::string name;
    PropertyInfo* info;
    Idx idx;
};

class PropertyKeyDB
{

public:

    ~PropertyKeyDB()
    {
        for(std::map<std::string, PropertyKeyDBEntry*>::iterator it = name_to_info_.begin(); it != name_to_info_.end(); ++it)
        {
            delete it->second;
        }
    }

    template<typename T>
    void registerProperty(const std::string& name, PropertyKey<T>& key, PropertyInfo* info = 0)
    {
        PropertyKeyDBEntry* entry;

        std::map<std::string, PropertyKeyDBEntry*>::iterator it = name_to_info_.find(name);
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

    const PropertyKeyDBEntry* getPropertyKeyDBEntry(const std::string& name) const
    {
        std::map<std::string, PropertyKeyDBEntry*>::const_iterator it = name_to_info_.find(name);
        if (it == name_to_info_.end())
            return 0;

        return it->second;
    }

private:

    std::map<std::string, PropertyKeyDBEntry*> name_to_info_;

};

} // end namespace

#endif
