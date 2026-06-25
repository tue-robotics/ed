#ifndef ED_ERROR_CONTEXT_H_
#define ED_ERROR_CONTEXT_H_

#include <vector>

namespace ed
{

struct ErrorContextData
{
    std::vector<std::pair<const char*, const char*>> stack;
};

class ErrorContext
{

public:
    ErrorContext(const char* msg, const char* value = nullptr);

    ~ErrorContext();

    static void change(const char* msg, const char* value = nullptr);

    static ErrorContextData* data();
};

} // end namespace ed

#endif
