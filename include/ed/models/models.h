#ifndef ED_MODELS_MODELS_H_
#define ED_MODELS_MODELS_H_

#include "ed/types.h"
#include <tue/config/configuration.h>

namespace ed
{

namespace models
{

bool create(UpdateRequest& req, const UUID& id, const TYPE& type, tue::Configuration cfg = tue::Configuration());

}

}

#endif
