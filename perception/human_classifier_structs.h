#ifndef HUMAN_CLASSIFIER_STRUCTS_H
#define HUMAN_CLASSIFIER_STRUCTS_H

// STL includes
#include <vector>


// OpenCV includes
#include <opencv/cv.h>
// #include <human_tracking/Regions.h>
// #include "Config.h"


//using namespace cv;


// ----------------------------------
// 			STRUTCTURES
// ----------------------------------

enum TemplateType {
    FaceFront = 0, FaceLeft = 1, FaceRight = 2, FaceBack = 3, NoMatch = -1
};

/**
 * Persistency memory structure
 */
struct SightInfo {
	float consecutive;	/*!< Error associated with that memory */
	int lastSeen;		/*!< Number of cycles without seeing the region */
	int sighID;
    std::vector<float> scores;
};

/**
 * Entry for the manual annotations, for testing
 */
struct AnnotationEntry {
	int nObjs;			/**< Number of objects present in the frame */
    std::vector<cv::Point> personLoc;	/*!< Person location */
};


/**
 * Structure for different color channels
 */
struct HistPlanes {
    cv::Mat hist_h;		/*!< Hue histogram */
    cv::Mat hist_s;		/*!< Saturation histogram */
    cv::Mat hist_v;		/*!< Value histogram */

    cv::Mat hist_r;		/*!< Red histogram */
    cv::Mat hist_g;		/*!< Green histogram */
    cv::Mat hist_b;		/*!< Blue histogram */
};


/**
 * Region information
 */
struct Roi{
	bool isHuman;		/**< Indicates if the regions is considered human or not */
	float score;		/**< Classification score for the current frame */
	float weightedScore;/**< Classification score weighted for several frames */
	int trackingID;		/**< Unique Tracking ID (only valid if is a human region) */
	int sightID;		/**< unique ID of the sighting, to relate regions over several frames */
    cv::Mat mask;			/**< Contour draw of the regions */
    std::vector<std::vector<cv::Point> > contour; /**< contour of the region */
    cv::Point3f centroid;	/**< Centroid of the region */
    cv::Point3i matchLocImg;	/**< Best match position in the image*/
    cv::Point3f matchLocWorld;	/**< Best match position in the world*/
	TemplateType templType;		/**< Number of the template that matched */
    cv::Rect template_box;	/**< Rectangle defined by the overlaped template */
	uint idStatus;		/**< Indicates if the user has already been identified */
    std::string personName;
    std::vector<cv::Rect> faceLocation;
    std::vector<cv::Point> leftArmPts;
    std::vector<cv::Point> rightArmPts;
    bool wavingLeft;
    bool wavingRight;
    float avgDepthVal;
};


/**
 * Tracking information
 */
struct TrackInfo {
	uint idStatus;		/**< Indicates if the user has already been identified */
	int trackingID;		/**< Unique Tracking ID */
	int lastSeen;		/**< Number of cycles without seeing the region */
	HistPlanes histogram;	/**< Histogram planes */
    std::string personName;
};


/**
 * Comparator for two Point3i. The smallest X coordinate comes first
 */
struct compPoint3i {
	/*!
	 * \brief Comparator between two Point3i points
	 * @param	a	first point
	 * @param	b	second point
	 * @return	true if the first point comes before the second point
	 */
    bool operator()(const cv::Point3f& a, const cv::Point3f& b) const {
		if (a.x <= b.x)
			return true;
		else
			return false;
	}
};

/*
struct PointComparator {
    bool operator() (cv::Point pt1, cv::Point pt2) {
        return (pt1.x < pt2.x);}
} SortPointsCustom;


struct PointComparatorReverse {
    bool operator() (cv::Point pt1, cv::Point pt2) {
        return (pt1.x > pt2.x);}
} SortPointsReverseCustom;
*/

// typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, human_tracking::Regions> MySyncPolicy;
typedef std::map<cv::Point3f, SightInfo>::iterator itMap;
typedef std::map<int, AnnotationEntry> Annotation;
typedef std::map<int, AnnotationEntry>::iterator itAnnot;


// ------------- GLOBAL -------------

// histogram calculation
// int 	histSize = 255;		/*!< histogram number of bins */
// float 	range[] = { 0, 256 };
// float 	s_range[] = { 50, 180 }; // 0, 180
// float	h_range[] = { 0, 256 };



#endif
