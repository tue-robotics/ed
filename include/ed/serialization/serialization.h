#ifndef ED_SERIALIZATION_H_
#define ED_SERIALIZATION_H_

#include "ed/types.h"

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

namespace ed
{

class ImageMask;

// SERIALIZATION

void serialize(const ImageMask& mask, tue::serialization::OutputArchive& m);

// DESERIALIZATION

void deserialize(tue::serialization::InputArchive& m, ImageMask& mask);

}

#endif
