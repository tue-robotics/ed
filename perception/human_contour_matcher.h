#ifndef ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_
#define ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_

#include <ed/perception_modules/perception_module.h>
#include "human_classifier.h"

class HumanContourMatcher : public ed::PerceptionModule
{

private:
    HumanClassifier human_classifier_;

    bool init_success_;

public:

    HumanContourMatcher();

    virtual ~HumanContourMatcher();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

};

#endif
