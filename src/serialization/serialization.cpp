#include "ed/serialization/serialization.h"
#include "ed/mask.h"

namespace ed
{

const static int MASK_SERIALIZATION_VERSION = 0;

// ----------------------------------------------------------------------------------------------------
//
//                                          SERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

void serialize(const ImageMask& mask, tue::serialization::OutputArchive& m)
{
    m << MASK_SERIALIZATION_VERSION;

    m << mask.width();
    m << mask.height();

    // Determine size
    int size = 0;
    for(ImageMask::const_iterator it = mask.begin(); it != mask.end(); ++it)
        ++size;

    m << size;

    for(ImageMask::const_iterator it = mask.begin(); it != mask.end(); ++it)
        m << it->y * mask.width() + it->x;
}

// ----------------------------------------------------------------------------------------------------
//
//                                         DESERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

void deserialize(tue::serialization::InputArchive& m, ImageMask& mask)
{
    int version;
    m >> version;

    int width, height;
    m >> width;
    m >> height;
    mask = ImageMask(width, height);

    int size;
    m >> size;

    for(int i = 0; i < size; ++i)
    {
        int idx;
        m >> idx;

        mask.addPoint(idx % width, idx / width);
    }
}

}
