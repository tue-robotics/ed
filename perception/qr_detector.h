#ifndef qr_detector_h_
#define qr_detector_h_

#include <ed/perception_modules/perception_module.h>

class QRDetector : public ed::PerceptionModule
{

public:

    QRDetector();

    virtual ~QRDetector();

    void configure(tue::Configuration config);

    void loadModel(const std::string& model_name, const std::string& model_path);

    ed::PerceptionResult process(const ed::Measurement& msr) const;

};

#endif
