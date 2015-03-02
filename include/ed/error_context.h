#ifndef ED_ERROR_CONTEXT_H_
#define ED_ERROR_CONTEXT_H_

#include <vector>

namespace ed
{

struct ErrorContextData
{
    std::vector<std::pair<const char*, const char*> > stack;
};

class ErrorContext
{

public:

    ErrorContext(const char* msg, const char* value = 0);

    ~ErrorContext();

    void change(const char* msg, const char* value = 0);

    static ErrorContextData* data();

};

} // end namespace ed

#endif
