#ifndef ED_TYPE_AGGREGATOR_H_
#define ED_TYPE_AGGREGATOR_H_

#include <ed/perception_modules/perception_module.h>

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
    float kPositiveTresh;

    void classify(std::vector<std::string> perceptionRes, std::string entityType) const;

};

#endif
