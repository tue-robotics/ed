#ifndef ED_TYPE_AGGREGATOR_H_
#define ED_TYPE_AGGREGATOR_H_

#include <ed/perception_modules/perception_module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

struct dictionary_match{
    uint matches;
    float score;
    std::string entry;
} ;

class TypeAggregator : public ed::PerceptionModule
{



/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/

public:

    TypeAggregator();

    virtual ~TypeAggregator();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;


/*
* ###########################################
*  				PRIVATE
* ###########################################
*/

private:

    // module configuration
    bool init_success_;
    std::string	kModuleName;    /*!< Name of the module, for output */

    std::vector<std::string> kPluginNames;
    std::map<std::string, std::vector<std::string> > dictionary;

    float kPositiveTresh;

    void collect_results(tue::Configuration &entity_conf,
                         std::map<std::string, std::map<std::string, float> >& hypothesis,
                         std::map<std::string, std::pair<std::string, float> >& features) const;

    std::string best_hypothesis(std::map<std::string, std::map<std::string, float> > hypothesis) const;

    bool load_dictionary(const std::string path) ;

    void match_dictonary(std::map<std::string, std::map<std::string, float> >& hypothesis,
                           std::map<std::string, std::pair<std::string, float> >& features,
                           std::string& type, float &certainty) const;
};

#endif
