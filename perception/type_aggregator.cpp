#include "type_aggregator.h"
#include "ed/measurement.h"
#include <ed/entity.h>
#include <algorithm>

#include <rgbd/Image.h>
#include <rgbd/View.h>

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
        kPluginNames.push_back("odu_finder");

        kPositiveTresh = 0.5;

        init_success_ = true;
    }
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    // if initialization failed, return
    if (!init_success_)
        return;

    /*
    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->bestMeasurement();
    if (!msr)
        return;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

    cv::Mat mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);
    // Iterate over all points in the mask
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        mask.at<unsigned char>(*it) = 255;
    }
    */

    float score = 0;
    std::string type = "";
    std::string label = "";
    std::map<std::string, std::map<std::string, float> > hypothesis;    // [hypothesis name | [plugin who named it | score]]
    std::map<std::string, std::map<std::string, float> >::iterator map_it;

    // find perception_result group
    if (result.readGroup("perception_result", tue::OPTIONAL))
    {
        // find perception plugins that already processed the entity
        for(std::vector<std::string>::const_iterator pluginName = kPluginNames.begin(); pluginName != kPluginNames.end(); ++pluginName) {
           if (result.readGroup(*pluginName)){

               // If it has a Score field, than it has a unique result
               if (result.value("score", score, tue::OPTIONAL))
               {
                   // if its bigger than the threshold, add that label
                   if (score > kPositiveTresh){
                       if (result.value("label", label)){
                           // if its not the first label, add a coma and then the new label
                           if (!type.empty()) type.append(", ");

                           type.append(label);
                       }
                   }
               }

               // If it has a Hypothesis field, than it has multiple result
               if (result.readArray("hypothesis", tue::OPTIONAL)){
//                   std::cout << "[" << kModuleName << "] " << "In " << *pluginName << std::endl;
                   // iterate through the hypothesis
                   while(result.nextArrayItem())
                   {
                       std::string hypothesis_name;
                       float hypothesis_score;
                       // if the hypothesis has a name and score
                       if (result.value("name", hypothesis_name) && result.value("score", hypothesis_score))
                       {
//                           std::cout << "[" << kModuleName << "] " << "Found: " << hypothesis_name << ", " << hypothesis_score << std::endl;
                           map_it = hypothesis.find(hypothesis_name);
                           // if hypothesis already exists, add the plugin name and score where it was found
                           if (map_it != hypothesis.end()){
//                               std::cout << "[" << kModuleName << "] " << "\t appending: " << *pluginName << ", " << hypothesis_score << std::endl;
                               map_it->second.insert(std::pair<std::string, float>(*pluginName, hypothesis_score));
                           }else{
//                               std::cout << "[" << kModuleName << "] " << "\t adding: " << hypothesis_name << ", " << hypothesis_score << std::endl;
                               // otherwise add a new entry
                               std::map<std::string, float> temp;
                               temp.insert(std::pair<std::string, float>(*pluginName, hypothesis_score));

                               hypothesis.insert(std::pair<std::string, std::map<std::string, float> >(hypothesis_name, temp));
                           }
                       }
                   }
                   result.endArray();
               }

               // close the group just read
               result.endGroup();
           }
        }
        // close perception_result group
        result.endGroup();

        // determine result from the hypothesis if any
        if (!hypothesis.empty()){
            // if its not the first label, add a coma and then the new label
            if (!type.empty()) type.append(", ");

            type.append(best_hypothesis(hypothesis));
        }

        if (!type.empty() && (type.find("Face") >= 0 || type.find("Human Shape") >= 0))
            result.setValue("type", "human");

//        cv::imwrite("/tmp/aggregator/" + e->id() + "_type_" + type + ".png", mask);
    }
    else{
//        std::cout << "[" << kModuleName << "] " << "perception_result group not found." << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

std::string TypeAggregator::best_hypothesis(std::map<std::string, std::map<std::string, float> > hypothesis) const{

    std::map<std::string, std::map<std::string, float> >::const_iterator it_outer;
    std::map<std::string, float>::const_iterator it_inner;
    float highest_score = 0;
    float final_score;
    std::string best_hypothesis = "";
    uint equal_results = 0;

    // iterate through all hypothesis
    for(it_outer = hypothesis.begin(); it_outer != hypothesis.end(); ++it_outer)
    {
        final_score = 0;
//        std::cout << it_outer->first << std::endl;

        // acumulate all the scores for this hypothesis
        for(it_inner = it_outer->second.begin(); it_inner != it_outer->second.end(); ++it_inner)
        {
//            std::cout << "\t " << it_inner->first << ", " << it_inner->second << std::endl;
            final_score += it_inner->second;
        }

        // save the best hypothesis
        if (highest_score < final_score){
            // update best score
            highest_score = final_score;
            best_hypothesis = it_outer->first;

            //reset counter
            equal_results = 0;
        }else if (highest_score == final_score){
            equal_results++;
        }
    }

    if (equal_results > 1)
        return "INCONCLUSIVE";
    else
        return best_hypothesis;
}


ED_REGISTER_PERCEPTION_MODULE(TypeAggregator)
