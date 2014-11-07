#ifndef ED_PERCEPTION_SIZE_MATCHER_H_
#define ED_PERCEPTION_SIZE_MATCHER_H_

#include <ed/perception_modules/perception_module.h>

namespace ed
{
namespace perception
{

struct ObjectSize
{
    double min_height;
    double max_height;
    double min_width;
    double max_width;

    ObjectSize(){}
    ObjectSize(double hmin, double hmax, double wmin, double wmax)
    {
        min_height = std::max(hmin, 0.0);
        max_height = hmax;
        min_width = std::max(wmin, 0.0);
        max_width = wmax;
    }

    // Update values
    void updateHMin(double hmin) {min_height = std::max(hmin, 0.0);}
    void updateHMax(double hmax) {max_height = hmax;}
    void updateWMin(double wmin) {min_width = std::max(wmin, 0.0);}
    void updateWmax(double wmax) {max_width = wmax;}
};

class SizeMatcher : public PerceptionModule
{

public:

    SizeMatcher();

    virtual ~SizeMatcher();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

private:

    std::map<std::string, std::vector<ObjectSize> > models_;

    float small_tresh;
    float medium_tresh;

    std::string	kModuleName;    /*!< Name of the module, for output */
};

}

}

#endif
