#ifndef ED_TYPE_AGGREGATOR_H_
#define ED_TYPE_AGGREGATOR_H_

#include <ed/perception_modules/perception_module.h>

template <typename T1, typename T2>
struct less_second {
    typedef std::pair<T1, T2> type;
    bool operator ()(type const& a, type const& b) const {
        return a.second < b.second;
    }
};

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

    std::string best_hypothesis(std::map<std::string, std::map<std::string, float> > hypothesis) const;
};

#endif
