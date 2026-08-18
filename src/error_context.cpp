#include "ed/error_context.h"

#include <bits/pthreadtypes.h>
#include <pthread.h>
#include <utility>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

namespace
{

void dataDestructor(void* data)
{
    auto const* edata = static_cast<ErrorContextData*>(data);
    delete edata;
}

struct KeyHolder
{

    KeyHolder() { pthread_key_create(&key, &dataDestructor); }

    pthread_key_t key{};
};

// The pthread key is process-wide by construction and is created/destroyed by this holder.
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
KeyHolder key;

} // namespace

// ----------------------------------------------------------------------------------------------------

ErrorContext::ErrorContext(const char* msg, const char* value)
{
    ErrorContextData* edata = data();
    if (!edata)
    {
        edata = new ErrorContextData;
        pthread_setspecific(key.key, edata);
    }

    edata->stack.emplace_back(msg, value);
}

ErrorContext::~ErrorContext()
{
    ErrorContextData* edata = data();
    if (!edata)
        return;

    edata->stack.pop_back();
}

void ErrorContext::change(const char* msg, const char* value)
{
    ErrorContextData* edata = data();
    edata->stack.back() = std::pair<const char*, const char*>(msg, value);
}

ErrorContextData* ErrorContext::data()
{
    return static_cast<ErrorContextData*>(pthread_getspecific(key.key));
}

} // namespace ed
