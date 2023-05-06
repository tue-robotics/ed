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
    ErrorContextData* _data = data();
    if (!_data)
    {
        _data = new ErrorContextData;
        pthread_setspecific(key.key, _data);
    }

    _data->stack.push_back(std::pair<const char*, const char*>(msg, value));

}

ErrorContext::~ErrorContext()
{
    ErrorContextData* _data = data();
    if (!_data)
        return;

    _data->stack.pop_back();
}

void ErrorContext::change(const char* msg, const char* value)
{
    ErrorContextData* _data = data();
    _data->stack.back() = std::pair<const char*, const char*>(msg, value);
}

ErrorContextData* ErrorContext::data()
{
    return static_cast<ErrorContextData*>(pthread_getspecific(key.key));
}

}
