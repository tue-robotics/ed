#include "ed/error_context.h"

#include <pthread.h>
#include <iostream>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

namespace
{

void dataDestructor(void* data)
{
    ErrorContextData* edata = static_cast<ErrorContextData*>(data);
    delete edata;
}

struct KeyHolder
{

    KeyHolder() {
        pthread_key_create(&key, &dataDestructor);
    }

    pthread_key_t key;

};

    static KeyHolder key;

}

// ----------------------------------------------------------------------------------------------------

ErrorContext::ErrorContext(const char* msg, const char* value)
{
    ErrorContextData* data = static_cast<ErrorContextData*>(pthread_getspecific(key.key));
    if (!data)
    {
        data = new ErrorContextData;
        pthread_setspecific(key.key, data);
    }

    data->stack.push_back(std::pair<const char*, const char*>(msg, value));

}

ErrorContext::~ErrorContext()
{
    ErrorContextData* data = static_cast<ErrorContextData*>(pthread_getspecific(key.key));
    if (!data)
        return;

    data->stack.pop_back();
}

void ErrorContext::change(const char* msg, const char* value)
{
    ErrorContextData* data = static_cast<ErrorContextData*>(pthread_getspecific(key.key));
    data->stack.back() = std::pair<const char*, const char*>(msg, value);
}

ErrorContextData* ErrorContext::data()
{
    return static_cast<ErrorContextData*>(pthread_getspecific(key.key));
}

}
