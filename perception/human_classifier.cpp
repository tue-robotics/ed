/*
 * human_classifier.cpp
 *
 *  Created on: Jul 11, 2014
 *      Author: Luis Ferreira
 */

#include "human_classifier.h"


HumanClassifier::HumanClassifier(const std::string& module_name) {
    kModuleName = module_name;
}


HumanClassifier::~HumanClassifier()
{
}

bool HumanClassifier::Classify(const cv::Mat& depth_img,
                               const cv::Mat& color_img,
                               const cv::Mat& mask,
                               float& avg_depth,
                               float& template_match_error,
                               float& template_match_deviation,
                               std::string& template_stance) const {

//    std::cout << "[" << kModuleName << "] " << "Classifying object" << std::endl;

    cv::Point3i match_pos, match_init_pos;
    float match_error;
    float match_variance;
    TemplateType template_type;
    Roi measurement;
    bool faceDetected = false;

    // try the template matching
    if (TemplateClassification(depth_img, mask, avg_depth, match_pos, match_init_pos, match_error, match_variance, template_match_deviation, template_type, measurement)){

//        std::cout << "[" << kModuleName << "] " << "Template matching result:" << std::endl;
        if (template_type == FaceFront){
            template_stance = "front";
//            std::cout << "[" << kModuleName << "] " << "\tStance: Front" << std::endl;
        }
        else if (template_type == FaceLeft){
            template_stance = "side_left";
//            std::cout << "[" << kModuleName << "] " << "\tStance: Left" << std::endl;
        }
        else if (template_type == FaceRight){
            template_stance = "side_right";
//            std::cout << "[" << kModuleName << "] " << "\tStance: Right" << std::endl;
        }
        else if (template_type == NoMatch){
            template_stance = "";
            template_match_error = 0;
            template_match_deviation = 0;
//            std::cout << "[" << kModuleName << "] " << "\tStance: No Match!" << std::endl;
        }

//        std::cout << "[" << kModuleName << "] " << "\tError: " << match_error << std::endl;
//        std::cout << "[" << kModuleName << "] " << "\tVariance: " << match_variance << std::endl;
//        std::cout << "[" << kModuleName << "] " << "\tDeviation: " << template_match_deviation << std::endl;
    }else{
//        std::cout << "[" << kModuleName << "] " << "Could not match any of the templates" << std::endl;
    }

    // try the face detection
    if (kFaceDetectEnabled)
        faceDetected = FaceDetection(color_img, measurement);

    template_match_error = match_error;
    if ((template_match_error < kMaxTemplateErr && template_type != NoMatch) || faceDetected){
        return true;
    }else{
        return false;
    }
}

bool HumanClassifier::TemplateClassification(const cv::Mat& depth_image,
                                             const cv::Mat& mask,
                                             float& avg_depth,
                                             cv::Point3i &match_pos,
                                             cv::Point3i &match_init_pos,
                                             float& match_error,
                                             float& match_variance,
                                             float& match_deviation,
                                             TemplateType &template_type,
                                             Roi &measurement) const{

    cv::Rect maskBox;
    cv::Mat map_dt;
    cv::Mat contour_line;
    cv::Mat mask_copy;

    mask.copyTo(measurement.mask);
    mask.copyTo(mask_copy);

    // ---------- IMAGE PRE-PROCESSING ----------

    // TODO dont creat a Mat mask, just give human_classifier_.Classify a vector of 2D points!
    // find external contours only
    findContours(mask_copy, measurement.contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // create and outlined contour
    contour_line = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_8UC1);
    drawContours(contour_line, measurement.contour, -1, cv::Scalar(255), kDtLineWidth);

    // create a minimum area bounding box
    maskBox = boundingRect(measurement.contour[0]);

    // invert the outline, from white to black
    threshold(contour_line(maskBox), contour_line, 250, 255, CV_THRESH_BINARY_INV);

    // create a border on the image
    copyMakeBorder(contour_line, contour_line, kBorderSize, kBorderSize, kBorderSize, kBorderSize,
            cv::BORDER_CONSTANT, cv::Scalar(255,255,255) );

    // create the distance transform map from the contour line
    distanceTransform(contour_line, map_dt, CV_DIST_L2, 5);


    // ---------- TEMPLATE MATCHING ----------

    std::vector<std::vector <cv::Point> > templates_resized;

    // Resize templates according to the depth of the object
    templates_resized.push_back(std::vector<cv::Point>());
    ResizeTemplate(kTemplatesOriginal[0], avg_depth, -5, FaceFront, templates_resized.back());

    templates_resized.push_back(std::vector<cv::Point>());
    ResizeTemplate(kTemplatesOriginal[1], avg_depth, -8, FaceLeft, templates_resized.back());

    templates_resized.push_back(std::vector<cv::Point>());
    ResizeTemplate(kTemplatesOriginal[2], avg_depth, -8, FaceRight, templates_resized.back());

    // initialize error and template type
    match_error = 0;
    measurement.templType = NoMatch;
    template_type = NoMatch;

    for (uint i = 0; i < templates_resized.size(); i++) {
        cv::Point3i current_loc;
        cv::Point3i current_init_loc;
        cv::Rect template_box_relative;
        float current_err;
        float current_variance;
        float current_deviation;

        // TODO decide if i should pass the info like error, variance, stance, all in the measurement or separate variables for all
        // match the template with the DT map
        if (PerfectMatch(measurement.mask(maskBox), map_dt, templates_resized[i], current_loc, current_init_loc, current_err, current_variance, current_deviation)){

//            std::cout << "[" << kModuleName << "] " << "Match error for templt " << i << ": " << current_err << std::endl;

            // update information regarding the matching, error, template type, etc
            if (match_error > current_err || measurement.templType == NoMatch){
                template_box_relative = boundingRect(templates_resized[i]);

                match_pos = current_loc;
                match_init_pos = current_init_loc;
                match_error = current_err;
                match_variance = current_variance;
                match_deviation = current_deviation;

                measurement.templType = static_cast<TemplateType>(i);
                template_type = static_cast<TemplateType>(i);

                measurement.template_box = cv::Rect (ClipInt(current_loc.x + maskBox.x - kBorderSize, 0, depth_image.cols),
                                                     ClipInt(current_loc.y + maskBox.y - kBorderSize, 0, depth_image.rows),
                                                     ClipInt(template_box_relative.width, 1, depth_image.rows - template_box_relative.width),
                                                     ClipInt(template_box_relative.height, 1, depth_image.cols - template_box_relative.height));

                measurement.matchLocImg = cv::Point3i(ClipInt(measurement.template_box.x + template_box_relative.width/2, 0, depth_image.cols),
                                                      ClipInt(measurement.template_box.y + template_box_relative.height/2, 0, depth_image.rows),
                                                      current_loc.z);
            }
        }
        else{
//            std::cout << "[" << kModuleName << "] " << "Unable to perform template matching, region too small for the template" << std::endl;
        }
    }


    // ---------- DEBUGGING ----------

    if (kDebugMode) {
        // save Distance Transform map with the template overlayed to the debug folder
        if (measurement.templType != NoMatch){
            std::string msrID;

            if (match_error < kMaxTemplateErr){
                msrID = GenerateID() + "_HUMAN";
            }else{
                msrID = GenerateID() + "_NOT";
            }


            cv::imwrite(kDebugFolder + msrID + "_debug1_Mask.png", mask);
            cv::imwrite(kDebugFolder + msrID + "_debug2_Contour_obj_1_.png", contour_line);
//            cv::imwrite(kDebugFolder + msrID + "_debug3_DistanceTransform_obj_1_.png", map_dt);

            // draw the template over the distance transform map

            // normalizes the image so that it becomes more visible
            normalize(map_dt, map_dt, 0.0, 256, cv::NORM_MINMAX);

            float angleRad = (match_pos.z / 180.0) * M_PI;

            // draw the template over the place where it matched
            TemplateType best_templ= measurement.templType;
            for (uint a = 0; a < templates_resized.at(best_templ).size(); a++) {
                // select point to be painted on the dt map, apply rotation transformations
                cv::Point pos(floor( match_pos.x +
                                 (templates_resized.at(best_templ)[a].x * cos(angleRad) - templates_resized.at(best_templ)[a].y * sin(angleRad))),
                              floor( match_pos.y +
                                 (templates_resized.at(best_templ)[a].x * sin(angleRad) + templates_resized.at(best_templ)[a].y * cos(angleRad))));

                // paint this pixel if its inside the image
                if (pos.x >= 0 && pos.y >= 0 && pos.x < map_dt.cols && pos.y < map_dt.rows) {
                    map_dt.at<float>(pos) = 255;
                }
            }

            // paint the initial and final position for the matching
            cvtColor(map_dt, map_dt, CV_GRAY2RGB);
            circle(map_dt, cv::Point(match_pos.x, match_pos.y), 2, cv::Scalar(50, 50, 255), CV_FILLED);
            circle(map_dt, cv::Point(match_init_pos.x, match_init_pos.y), 2, cv::Scalar(100, 255, 100), CV_FILLED);
            cv::imwrite(kDebugFolder + msrID + "_debug4_Match_Location.png", map_dt);
        }
    }


    // check if the matching was sucessfull or not
    if (measurement.templType == NoMatch){
        match_pos = cv::Point3i(0,0,0);
        match_init_pos = cv::Point3i(0,0,0);
        match_error = 0;
        match_variance = 0;
        match_deviation = 0;

        return false;
    }else{
        return true;
    }
}

bool HumanClassifier::PerfectMatch(const cv::Mat& mask,
                                   cv::Mat &map_dt,
                                   std::vector<cv::Point> template_pts,
                                   cv::Point3i& best_pos,
                                   cv::Point3i& start_pos,
                                   float& best_error,
                                   float& variance,
                                   float& deviation) const{

    float err;              // error value
    int occupancy_best;
    int occupancy;
    int start_col;
    cv::Mat grad_x;			// sobel gradient in X
    cv::Mat grad_y;			// sobel gradient in Y
    cv::Vec3f curr_grad;	// gradient value
    cv::Vec3f pos;			// position to test
    cv::Vec3f prev_grad;	// old gradient value
    cv::Vec3f step_width;	// (jump distance in X, jump distance in Y)
    cv::Vec3f upper_limits;	// limits for the position tests
    cv::Vec3f lower_limits;	// limits for the position tests
    std::vector<float> err_values;
    std::vector<float> curr_err_values;
    cv::Rect templtBox;
    cv::Rect colCounter;
    cv::Scalar meanErr;
    cv::Scalar devErr;

    templtBox = boundingRect(template_pts);

    // discard regions that are too small for the given template
    if (map_dt.rows - 2*kBorderSize < templtBox.height || map_dt.cols - 2*kBorderSize < templtBox.width) {
        best_pos = cv::Point3i(0,0,0);
        start_pos = cv::Point3i(0,0,0);
        best_error = 0;
        variance = 0;
        deviation = 0;
        return false;
    }

    // Initializations
    prev_grad = cv::Vec3f(0.0, 0.0, 0.0);
    step_width = cv::Vec3f(1.0, 1.0, 0.05);
    upper_limits = cv::Vec3f(map_dt.cols,map_dt.rows, M_PI);
    lower_limits = cv::Vec3f(0, 0, -M_PI);
    err = best_error = std::numeric_limits<float>::max();	// initial error
    occupancy_best = 0;
    start_col = 0;
    colCounter = cv::Rect(0,0, mask.cols / kNumSlicesMatching, mask.rows);
    variance = 0;

    // calculate best start position, divide the regions in vertical sections and choose the most occupied
    for (int i = 0; i < kNumSlicesMatching; i++) {
        // count number of non-black pixels of the current region
        occupancy = cv::countNonZero( mask(cv::Rect((mask.cols / kNumSlicesMatching) * i, 0, (mask.cols / kNumSlicesMatching), mask.rows)));

        if (occupancy > occupancy_best) {
            occupancy_best = occupancy;
            // zone = midle of the zone - offset of half the template
            start_col = ClipInt(floor(((mask.cols / kNumSlicesMatching) * i) - (templtBox.width/2)), 0, mask.cols);
        }
    }

    // set intial position, add borderSize or remove it for original location
    pos = cv::Vec3f(start_col + kBorderSize, kBorderSize, 0.0);
    start_pos = cv::Point3i(start_col + kBorderSize, kBorderSize, 0.0);

    // Gradient in X
    Sobel(map_dt, grad_x, CV_32FC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    // Gradient in Y
    Sobel(map_dt, grad_y, CV_32FC1, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);


    //---------------------------------------------------------


    // RPROP algorithm
    for (int i = 0; i < kMatchIterations; i++) {
        curr_err_values.clear();
        err = ErrorFunction(grad_x, grad_y, map_dt, template_pts, pos, curr_grad, curr_err_values);

        // paint the path taken by the template in the map, point(0,0) should be close to "white"
        if (kDebugMode){
            map_dt.at<float>(cv::Point(pos[0], pos[1])) = map_dt.at<float>(0,0);
        }

        // sometimes it happened, now it appears to be fixed, but just in case
        if (err < 0)
             std::cout << "[" << kModuleName << "] " << "Incorrect value for error (" << err << ")\n" << std::endl;

        // update best error
        if (err < best_error && err > 0) {
            err_values.clear();

            // backup error values for the best match, to calculate mean and deviation later
            for (uint x = 0; x < curr_err_values.size(); x++)
                err_values.push_back(curr_err_values[x]);

            // backup best error
            best_error = err;

            // update the pos to go from mapDT location to mask location
            best_pos = cv::Point3i(floor(pos[0]), floor(pos[1]), floor((pos[2]*180.0)/M_PI));
        }

        // move template in x, y, and teta (rotation)
        for (uint j = 0; j < 3; j++) {
            // make updates for each parameter
            if (curr_grad[j] == 0)
                prev_grad[j] = 0;
            else {
                // Increment adjustment
                if (curr_grad[j] * prev_grad[j] > 0)
                    step_width[j] *= 1.2;
                else if (curr_grad[j] * prev_grad[j] < 0)
                    step_width[j] *= 0.5;

                // backup last gradient result
                prev_grad[j] = curr_grad[j];

                // Adjustment of the parameters if it doesn't move the template outside the image
                if (curr_grad[j] > 0 && (pos[j] - step_width[j]) > lower_limits[j])
                    pos[j] -= step_width[j];
                else if (curr_grad[j] < 0 && (pos[j] + step_width[j]) < upper_limits[j])
                    pos[j] += step_width[j];
            }
        }

        // fitting is finished when the templates barelly moves in any direction
        if (step_width[0] < 0.3 && step_width[1] < 0.3 && step_width[2] < 0.2){
            break;
        }
    }

    // get mean error and deviation
    meanStdDev(err_values, meanErr, devErr);

    // calculate error variance
    for (uint i = 0; i < err_values.size(); ++i){
        variance += pow(err_values[i] -  meanErr[0], 2);
    }

    variance /= err_values.size();
    deviation = devErr[0];
    best_error = meanErr[0];

    return true;
}


float HumanClassifier::ErrorFunction(const cv::Mat& grad_x,
                                     const cv::Mat& grad_y,
                                     const cv::Mat& distance_transf,
                                     std::vector<cv::Point>& template_pts,
                                     cv::Vec3f& anchor,
                                     cv::Vec3f& grad,
                                     std::vector<float>& err_values) const{

    cv::Point pos;
    float err = 0;
    float gx;
    float gy;
    float d;
    std::vector<cv::Point> pos_tested;
    float anchor_sin = sin(anchor[2]);
    float anchor_cos = cos(anchor[2]);

    // reset the gradient value
    grad = cv::Vec3f(0.0, 0.0, 0.0);

    // test all template points
    for (uint i = 0; i < template_pts.size(); i++){

        // set testing position, with transformation (rotation)
        pos.x = ClipInt(floor(anchor[0] + (template_pts[i].x * anchor_cos - template_pts[i].y * anchor_sin) ),
                             0, distance_transf.cols-1);

        pos.y = ClipInt(floor(anchor[1] + (template_pts[i].x * anchor_sin + template_pts[i].y * anchor_cos) ),
                             0 , distance_transf.rows-1);

        // avoid points that have already been tested
        if (find(pos_tested.begin(), pos_tested.end(), pos) != pos_tested.end()) {
            //ROS_INFO("Already tested (%d, %d)", pos.x, pos.y);
        } else {
            gx = grad_x.at<float>(pos);
            gy = grad_y.at<float>(pos);
            d = distance_transf.at<float>(pos) * distance_transf.at<float>(pos);

            grad[0] += gx;	// gradient in X
            grad[1] += gy;	// gradient in Y
            grad[2] += gx * (-template_pts[i].x * anchor_sin - template_pts[i].y * anchor_cos)
                    + gy * (template_pts[i].x * anchor_cos + template_pts[i].y * anchor_sin);

            err += d;	// error for this position, or "distance"

            err_values.push_back(d);
            pos_tested.push_back(pos);
        }

        // a warning just in case..
        if (pos.x > distance_transf.cols || pos.y > distance_transf.rows || pos.x < 0 || pos.y < 0)
            std::cout << "[" << kModuleName << "] " << "Position outside of the map ("<< pos.x << ", " << pos.y <<")" << std::endl;
    }

    return err;
}


bool HumanClassifier::FaceDetection(const cv::Mat &color_img, Roi &measurement) const{

    // if face detection is enabled, and the template matching didnt fail
    if (kFaceDetectEnabled && measurement.templType != NoMatch){

        std::vector<cv::Rect> facesFront;
        std::vector<cv::Rect> facesProfile;
        cv::Mat cascadeImg;
        cv::Rect headArea;
        bool faceDetected;

        OptimizeHeadRoi(measurement.template_box, measurement.templType, color_img, headArea);

        // prepare image for detection
        cvtColor(color_img(headArea), cascadeImg, CV_BGR2GRAY);
        normalize(cascadeImg, cascadeImg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        // detect faces
        cv::CascadeClassifier kDetectFaceFront;
        cv::CascadeClassifier kDetectFaceProfile;
        if (!kDetectFaceFront.load(cascade_path_ + "haarcascade_frontalface_default.xml") ||
                !kDetectFaceProfile.load(cascade_path_ + "haarcascade_profileface.xml")) {
            std::cout << "[" << kModuleName << "] " << "Unable to load all haar cascade files" << std::endl;
            return false;
        } else {
//            std::cout << "[" << kModuleName << "] " << "Haar cascade XML files sucessfully loaded." << std::endl;
        }

        kDetectFaceFront.detectMultiScale(cascadeImg, facesFront, 1.2, 2, 0|CV_HAAR_SCALE_IMAGE);

        // only search profile faces if the frontal face detection failed
        if (facesFront.size() == 0){
            kDetectFaceProfile.detectMultiScale(cascadeImg, facesProfile, 1.1, 1, 0|CV_HAAR_SCALE_IMAGE);
        }

        // confirm the candidate is human if a face is detected
        faceDetected = (facesFront.size() > 0 || facesProfile.size() > 0);

        // save the ROIs of the faces found
        for (uint j = 0; j < facesFront.size(); j++) {
            facesFront[j].x += headArea.x;
            facesFront[j].y += headArea.y;
            measurement.faceLocation.push_back(cv::Rect(facesFront[j]));
        }

        for (uint j = 0; j < facesProfile.size(); j++) {
            facesProfile[j].x += headArea.x;
            facesProfile[j].y += headArea.y;
            measurement.faceLocation.push_back(cv::Rect(facesProfile[j]));
        }

        if (kDebugMode && faceDetected){
            cv::Mat debugImg;

            color_img.copyTo(debugImg);

            for (uint j = 0; j < facesFront.size(); j++)
                cv::rectangle(debugImg, facesFront[j], cv::Scalar(0, 255, 0), 2, CV_AA);

            for (uint j = 0; j < facesProfile.size(); j++)
                cv::rectangle(debugImg, facesProfile[j], cv::Scalar(0, 0, 255), 2, CV_AA);

            cv::imwrite(kDebugFolder + GenerateID() + "_debug5_Face_Detection.png", debugImg);
        }

        if (faceDetected)
            return true;
        else
            return false;
    }

    return false;
}

void HumanClassifier::OptimizeHeadRoi(cv::Rect& templateArea, TemplateType templateN, const cv::Mat &color_img, cv::Rect& headArea) const{

    // create a copy of the template matching area
    headArea = cv::Rect(templateArea);

    // increse the size of the head area and center it better
    if (templateN == FaceFront){
        headArea.y = ClipInt (floor(headArea.y - headArea.height * 0.2), 0, color_img.rows);

        headArea.width = ClipInt (headArea.width * 1.2, 0, color_img.cols - headArea.x);
        headArea.height = ClipInt (headArea.height * 1.2, 0, color_img.rows - headArea.y);

    }else if(templateN == FaceLeft){
        headArea.x = ClipInt (floor(headArea.x - headArea.width * 0.3), 0, color_img.cols);
        headArea.y = ClipInt (floor(headArea.y - headArea.height * 0.2), 0, color_img.rows);

        headArea.width = ClipInt (headArea.width * 1.3, 0, color_img.cols - headArea.x);
        headArea.height = ClipInt (headArea.height * 1.2, 0, color_img.rows - headArea.y);
    }
    else if (templateN == FaceRight){
        headArea.x = ClipInt (floor(headArea.x - headArea.width * 0.1), 0, color_img.cols);
        headArea.y = ClipInt (floor(headArea.y - headArea.height * 0.2), 0, color_img.rows);

        headArea.width = ClipInt (headArea.width * 1.3, 0, color_img.cols - headArea.x);
        headArea.height = ClipInt (headArea.height * 1.2, 0, color_img.rows - headArea.y);
    }
}


float HumanClassifier::ResizeTemplate(std::vector<cv::Point> template_src,
                                      float depth,
                                      float alpha,
                                      TemplateType type,
                                      std::vector<cv::Point>& template_dst) const{

    float scaleFactor;

    if (depth <= 0)
        std::cout << "[" << kModuleName << "] " << "Incorrect depth value = "<< depth << std::endl;

    // scale_factor = -15,763 X + 106,33 (linear)
    // scale_factor = -52.6 * log(X) + 114.06 (logarithmic)
    // scale_factor = 308 * X^(-1.018)
    // scaleFactor = (308 * pow(depth, -1.018)) + alpha;

    // use different scaling functions for different template types
    if (type == FaceFront)
        scaleFactor = (359.29 * pow(depth, -1.154)) + alpha;
    else if (type == FaceLeft || type == FaceRight)
        scaleFactor = (299.07 * pow(depth, -1.156)) + alpha;
    else
        scaleFactor = (412.21 * pow(depth, -1.256)) + alpha;

    // apply the scalling to each point
    for (uint j = 0; j < template_src.size(); j++)
        template_dst.push_back(	cv::Point(
                floor(template_src[j].x * scaleFactor/100),
                floor(template_src[j].y * scaleFactor/100)));

    return scaleFactor;
}

int HumanClassifier::ClipInt(int val, int min, int max) const{
    return val <= min ? min : val >= max ? max : val;
}


bool HumanClassifier::Initializations(const std::string& model_name, const std::string& module_path) {

    std::string template_path;

    kModelName = model_name;
    cascade_path_ = module_path + "cascade_classifiers/";
    template_path = module_path + "head_templates/";

    LoadParameters();

	// clean/create debug folder
    if (kDebugMode) CleanDebugFolder(kDebugFolder);

    // Load templates by the same order you enumerate them in "enum TemplateType"
    kTemplatesOriginal.clear();
    if ( !LoadTemplate(template_path + "template_front_4_trimmed.png", kTemplatesOriginal) ||
         !LoadTemplate(template_path + "left_close.png", kTemplatesOriginal) ||
         !LoadTemplate(template_path + "right_close.png", kTemplatesOriginal))
    {
        std::cout << "[" << kModuleName << "] " << "Unable to load all templates" << std::endl;
        return false;
    }
    else
        std::cout << "[" << kModuleName << "] " << "Templates sucessfully loaded" << std::endl;

    // initialize structuing element for morphological operations
    kMorphElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4), cv::Point(-1, -1));

    return true;
}


bool HumanClassifier::LoadTemplate(const std::string& template_path, std::vector<std::vector<cv::Point> >& template_list){
    cv::Mat template_img;
    std::vector<std::vector<cv::Point> > templateContours;

    template_img = cv::imread(template_path, CV_LOAD_IMAGE_GRAYSCALE);

	if (!template_img.data){
        std::cout << "[" << kModuleName << "] " << "Could not load the template " << template_path << std::endl;
        return false;
	}

	// create binary image, pixels > 50 = 1
	template_img = template_img > 50;
    findContours(template_img, templateContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    template_list.push_back(std::vector<cv::Point>());

	// copy and reduce total number of points in the contour
	for (uint i = 0; i < templateContours[0].size(); i+=2) {
		template_list.back().push_back(templateContours[0][i]);
	}

    std::cout << "[" << kModuleName << "] " << "Template loaded: " << template_path << ", " << template_list.back().size() << " points" << std::endl;

    return true;
}


void HumanClassifier::LoadParameters(){
    kDebugFolder = "/tmp/human_classifier/";

    kDebugMode = false;
    kFaceDetectEnabled = false;
    kMatchIterations = 30;
    kDtLineWidth = 1;
    kMaxTemplateErr = 15;
    kBorderSize = 20;
    kNumSlicesMatching = 7;
}


void HumanClassifier::CleanDebugFolder(const std::string& folder){
    std::cout << "[" << kModuleName << "] " << "Cleaning debug/output folders. Don't worry about error comments like 'rm: cannot remove'" << std::endl;

    if (system(std::string("mkdir " + folder + " > nul").c_str()) != 0){
		//printf("\nUnable to create output folder. Already created?\n");
	}
    if (system(std::string("rm " + folder + "*.png" + " > nul").c_str()) != 0){
		//printf("\nUnable to clean output folder \n");
	}
}


std::string HumanClassifier::GenerateID() const{
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    std::string ID;
    for (int i = 0; i < 10; ++i) {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        ID += alphanum[n];
    }

    return ID;
}
