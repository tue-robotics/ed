#ifndef ED_PERCEPTION_MODEL_FITTER_H_
#define ED_PERCEPTION_MODEL_FITTER_H_

#include <ed/perception_modules/perception_module.h>

#include <geolib/datatypes.h>

namespace ed
{

namespace model_fitter
{

//class ModelFitter : public ed::PerceptionModule
//{

//public:

//    ModelFitter();

//    virtual ~ModelFitter();

//    void loadModel(const std::string& model_name, const std::string& model_path);

//    void configure(tue::Configuration config);

//    ed::PerceptionResult process(const ed::Measurement& msr) const;

//private:

//    int sample_width_, sample_height_;

//    std::map<std::string, geo::ShapeConstPtr> models_;

//};

bool fit(const Entity& e, const geo::ShapeConstPtr& shape, geo::Pose3D& pose);

}

}

#endif
