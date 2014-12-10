#ifndef ED_MODELS_MODELS_H_
#define ED_MODELS_MODELS_H_

#include "ed/types.h"
#include <tue/config/configuration.h>

namespace ed
{

namespace models
{

bool create(const UUID& id, const std::string& type, UpdateRequest& req);

bool create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id, UpdateRequest& req, const std::string& model_path = "");

}

}

#endif
