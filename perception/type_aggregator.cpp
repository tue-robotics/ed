#include "type_aggregator.h"
#include "ed/measurement.h"
#include <ed/entity.h>
#include <algorithm>

#include <rgbd/Image.h>
#include <rgbd/View.h>


// ----------------------------------------------------------------------------------------------------

TypeAggregator::TypeAggregator():
    PerceptionModule("type_aggregator")
{
}

// ----------------------------------------------------------------------------------------------------

TypeAggregator::~TypeAggregator()
{
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::loadConfig(const std::string& config_path) {
    kModuleName = "type_aggregator";

    kPluginNames.push_back("human_contour_matcher");
    kPluginNames.push_back("face_detector");
    kPluginNames.push_back("size_matcher");
    kPluginNames.push_back("odu_finder");
    kPluginNames.push_back("color_matcher");

    kPositiveTresh = 0.5;

    std::cout << "[" << kModuleName << "] " << "Loading dictionary ..." << config_path + "/type_dictionary.yml" << std::endl;

    if (load_dictionary(config_path + "/type_dictionary.yml"))
        std::cout << "[" << kModuleName << "] " << "Ready!" << std::endl;
    else
        std::cout << "[" << kModuleName << "] " << "Unable to load dictionary from " << config_path + "/type_dictionary.yml" << std::endl;
        return;

    init_success_ = true;
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::process(ed::EntityConstPtr e, tue::Configuration& entity_conf) const
{

    // if initialization failed, return
//    if (!init_success_)
//        return;

    std::map<std::string, std::map<std::string, float> > hypothesis;   // [label | [plugin who named it | score]]
    std::map<std::string, std::pair<std::string, float> > features;    // [hypothesis name | [plugin who named it | score]]
    std::map<std::string, float> type_histogram;
    std::string type = "";
    float certainty = 0;

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

    // rebuild histogram from previous configuration
    if (entity_conf.readGroup("perception_result", tue::OPTIONAL))
    {
        if (entity_conf.readArray("histogram", tue::OPTIONAL))
        {
            while(entity_conf.nextArrayItem())
            {
                std::string type;
                float amount;

                if (entity_conf.value("type", type, tue::OPTIONAL) && entity_conf.value("amount", amount, tue::OPTIONAL))
                {
                    type_histogram.insert(std::pair<std::string, float>(type, 1));
                }else{
                    std::cout << "[" << kModuleName << "] " << "Malformed histogram entry." << type << ", " << amount << std::endl;
                }
            }
            entity_conf.endArray();
        }
        entity_conf.endGroup();
    }

    // collect features asserted by other perception plugins
    collect_features(entity_conf, features);

    // match these features against the features dictionary
    match_features(features, type_histogram, type, certainty);

    // assert type
//    if (!type.empty() && certainty > 0.5){
//        entity_conf.setValue("type", type);
//    }

    // create or read perception_result group
    if (!entity_conf.readGroup("perception_result", tue::OPTIONAL))
    {
        entity_conf.writeGroup("perception_result");
    }

    // Update histogram in the configuration
    entity_conf.writeArray("histogram");
    for(std::map<std::string, float>::const_iterator it = type_histogram.begin(); it != type_histogram.end(); ++it)
    {
        entity_conf.addArrayItem();
        entity_conf.setValue("type", it->first);
        entity_conf.setValue("amount", it->second);
        entity_conf.endArrayItem();
    }
    entity_conf.endArray(); // close histogram array

    entity_conf.writeGroup("type_aggregator");
    // assert type
    if (!type.empty() && certainty > 1){
        entity_conf.setValue("type", type);
        entity_conf.setValue("certainty", certainty);

//        std::cout << "[" << kModuleName << "] " << "Asserted type: " << type << " (" << certainty << ")" << std::endl;
    }else{
//        std::cout << "[" << kModuleName << "] " << "No hypothesis found." << std::endl;
    }

    entity_conf.endGroup(); // close type_aggregator group
    entity_conf.endGroup(); // close perception_result group
}


// ----------------------------------------------------------------------------------------------------


void TypeAggregator::match_features(std::map<std::string, std::pair<std::string, float> >& features,
                                    std::map<std::string, float>& type_histogram,
                                    std::string& type,
                                    float& amount) const{

    std::map<std::string, std::pair<std::string, float> >::const_iterator feat_it;
    std::map<std::string, std::vector<std::string> >::const_iterator dict_it;
    std::map<std::string, float>::iterator hist_it;

    // iterate through all dictionary entries
    for(dict_it = dictionary.begin(); dict_it != dictionary.end(); ++dict_it){
        std::string dictionary_entry = dict_it->first;

        // iterate through all features of a particular dictionary entry
        for(std::vector<std::string>::const_iterator dict_feat = dict_it->second.begin(); dict_feat != dict_it->second.end(); ++dict_feat) {

            // find matches with features
            for(feat_it = features.begin(); feat_it != features.end(); ++feat_it)
            {
                float feat_score = feat_it->second.second;
                if (feat_it->first.compare(*dict_feat) == 0 && feat_score > 0)
                {
                    // search for match with the dictionary
                    hist_it = type_histogram.find(dictionary_entry);

                    // add a new entry or update the existing one
                    if (hist_it != type_histogram.end()){
//                        std::cout << "[" << kModuleName << "] " << "Update entry: " << dictionary_entry << ", " <<
//                                     hist_it->second  << " + " << feat_score << std::endl;
                        hist_it->second += feat_score;
                    }else{
//                        std::cout << "[" << kModuleName << "] " << "New entry: " << dictionary_entry << ", " << feat_score << std::endl;
                        if (feat_score < 1) feat_score += 1;
                        type_histogram.insert(std::pair<std::string, float>(dictionary_entry, feat_score));
                    }
                }
            }
        }
    }

    // find best result in the hystogram
    for(hist_it = type_histogram.begin(); hist_it != type_histogram.end(); ++hist_it) {
        if (amount < hist_it->second){
            amount = hist_it->second;
            type = hist_it->first;
        }
    }
}


// ----------------------------------------------------------------------------------------------------

void TypeAggregator::collect_features(tue::Configuration& entity_conf, std::map<std::string, std::pair<std::string, float> >& features) const{


    float score = 0;
    std::string feat_name = "";
    std::map<std::string, std::map<std::string, float> >::iterator map_it;

    // find perception_result group
    if (entity_conf.readGroup("perception_result"))
    {
        // find perception plugins that already processed the entity
        for(std::vector<std::string>::const_iterator pluginName = kPluginNames.begin(); pluginName != kPluginNames.end(); ++pluginName) {
            // visit every perception plugin
            if (entity_conf.readGroup(*pluginName)){

                // collect Features
                if (entity_conf.value("score", score, tue::OPTIONAL) && entity_conf.value("label", feat_name, tue::OPTIONAL))
                {
                    std::pair<std::string, float> inner_pair (*pluginName, score);
                    features.insert(std::pair<std::string, std::pair<std::string, float> >(feat_name, inner_pair));
//                    std::cout << "Feature " << feat_name << ", from " << *pluginName << " with " << score << std::endl;
                }

                // collect Hypothesis
                if (entity_conf.readArray("hypothesis", tue::OPTIONAL)){
                    // iterate through the hypothesis
                    while(entity_conf.nextArrayItem())
                    {
                        // if the hypothesis has a name and score
                        if (entity_conf.value("name", feat_name, tue::OPTIONAL) && entity_conf.value("score", score, tue::OPTIONAL))
                        {
                            std::pair<std::string, float> inner_pair (*pluginName, score);
                            features.insert(std::pair<std::string, std::pair<std::string, float> >(feat_name, inner_pair));
//                            std::cout << "Feature " << feat_name << ", from " << *pluginName << " with " << score << std::endl;
                        }
                    }
                    entity_conf.endArray();
                }

                // close the group just read
                entity_conf.endGroup();
            }else{
                std::cout << "Didnt find group " << *pluginName << std::endl;
            }
        }

        // close perception_result group
        entity_conf.endGroup();
    }
}

// ----------------------------------------------------------------------------------------------------

bool TypeAggregator::load_dictionary(const std::string path) {
    if (path.empty()){
        std::cout << "[" << kModuleName << "] " << "Dictionary path not specified." << std::endl;
        return false;
    }else{
        tue::Configuration dictionary_conf;
        std::string type_entry;
        std::string feature_name;
        std::vector<std::string> features;

        // load list of models to include
        if (dictionary_conf.loadFromYAMLFile(path)){

            if (dictionary_conf.readArray("conclusions")){
                while(dictionary_conf.nextArrayItem())
                {
                    if (dictionary_conf.value("type", type_entry))
                    {
                        features.clear();
                        if (dictionary_conf.readArray("features"))
                        {
                            while(dictionary_conf.nextArrayItem())
                            {
                                if (dictionary_conf.value("name", feature_name))
                                {
                                    features.push_back(feature_name);
                                }
                            }
                        }
                        dictionary_conf.endArray();
                        dictionary.insert(std::pair<std::string, std::vector<std::string> >(type_entry, features));
                    }
                }
                dictionary_conf.endArray();
            }else{
                std::cout << "[" << kModuleName << "] " << "Dictionary incorrectly built." << std::endl;
                std::cout << dictionary_conf.error() << std::endl;
                return false;
            }
        }else{
            std::cout << "[" << kModuleName << "] " << "Dictionary not found at " << path << "." << std::endl;
            std::cout << dictionary_conf.error() << std::endl;
            return false;
        }
    }
    return true;
}

ED_REGISTER_PERCEPTION_MODULE(TypeAggregator)
