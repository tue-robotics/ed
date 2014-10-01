#ifndef ED_PERCEPTION_MODULE_H_
#define ED_PERCEPTION_MODULE_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PERCEPTION_MODULE(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::PerceptionModule)

#include "ed/types.h"

// Configuration
#include <tue/config/configuration.h>

namespace ed
{

//struct Percept
//{
//    Percept() {}

//    Percept(const std::string& label_, double prob, const geo::Pose3D& pose_)
//        : label(label_), score(prob), pose(pose_) {}

//    std::string label;
//    double score;
//    geo::Pose3D pose;

//};

//class PerceptionResult
//{

//public:

////    PerceptionResult() : ok(false) {}

//    void addInfo(const std::string& label, double prob, const geo::Pose3D& pose = geo::Pose3D::identity())
//    {
//        percepts_[label] = Percept(label, prob, pose);
//    }

//    const std::map<std::string, Percept>& percepts() const { return percepts_; }

//    void setVisualization(const std::string& name, const cv::Mat& visualization_image)
//    {
//        visualization_images_[name] = visualization_image;
//    }

//    const std::map<std::string, cv::Mat>& visualizationImages() const { return visualization_images_; }

//private:

//    std::map<std::string, Percept> percepts_;

//    // For visualization / debugging purpose
//    std::map<std::string, cv::Mat> visualization_images_;

//};

class PerceptionModule
{

public:

    PerceptionModule(const std::string& name) : name_(name) {}

    virtual ~PerceptionModule() {}

    virtual void loadModel(const std::string& model_name, const std::string& model_path) {}

    virtual void loadConfig(const std::string& config_path) {}

    virtual void configure(tue::Configuration config) {}

    virtual void process(EntityConstPtr e, tue::Configuration& result) const {}

//    virtual PerceptionResult process(const std::vector<MeasurementConstPtr>& measurements) const;

//    virtual PerceptionResult process(const Measurement& msr) const { return PerceptionResult(); }

    const std::string& name() const { return name_; }

private:

    std::string name_;

};

PerceptionModulePtr loadPerceptionModule(class_loader::ClassLoader* loader);

}

#endif
