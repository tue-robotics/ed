#include "type_aggregator.h"


#include "ed/measurement.h"
#include <ed/entity.h>

// ----------------------------------------------------------------------------------------------------

TypeAggregator::TypeAggregator():
    PerceptionModule("type_aggregator"),
    init_success_(false)
{
}

// ----------------------------------------------------------------------------------------------------

TypeAggregator::~TypeAggregator()
{
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::loadModel(const std::string& model_name, const std::string& model_path)
{
    if (model_name.compare("aggregator") == 0){

        kModuleName = "type_aggregator";

        kPluginNames.push_back("human_contour_matcher");
        kPluginNames.push_back("face_detector");
        kPluginNames.push_back("size_matcher");
        kPositiveTresh = 0.5;

        std::cout << "[" << kModuleName << "] " << "Finished loading model" << std::endl;

        init_success_ = true;
    }
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    // if initialization failed, return
    if (!init_success_)
        return;

    float score = 0;
    std::string type = "";
    std::string label = "";

    // find perception_result group
    if (result.readGroup("perception_result"))
    {
        // find perception plugins already executed
        for(std::vector<std::string>::const_iterator pluginName = kPluginNames.begin(); pluginName != kPluginNames.end(); ++pluginName) {
           if (result.readGroup(*pluginName)){
               // read the score
               if (result.value("score", score))
               {
                   // if its bigger than the threshold, add that label
                   if (score > kPositiveTresh){
                       if (result.value("label", label)){
                           // if its not the first label, add a coma and then the new label
                           if (!type.empty())
                               type.append(", ");

                           type.append(label);
                       }
                   }
               }
               result.endGroup();
           }
        }

        result.endGroup();

        // if no type was found, then its unknown
        if (type.empty())
            type = "Unknown";

        result.setValue("type", type);
//        std::cout << "[" << kModuleName << "] " << "type: " << type << std::endl;
    }
    else{
//        std::cout << "[" << kModuleName << "] " << "perception_result group not found." << std::endl;
    }
}


void TypeAggregator::classify(std::vector<std::string> perceptionRes, std::string entityType) const{

}



ED_REGISTER_PERCEPTION_MODULE(TypeAggregator)
