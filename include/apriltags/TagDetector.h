#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "apriltags//TagDetection.h"
#include "apriltags//TagFamily.h"
#include "apriltags//FloatImage.h"
#include <memory>

namespace AprilTags {

class TagDetector {
public:
	
	const TagFamily thisTagFamily;

	//! Constructor
  // note: TagFamily is instantiated here from TagCodes
	TagDetector(const TagCodes& tagCodes, const size_t blackBorder=2) : thisTagFamily(tagCodes, blackBorder) {}
	
	std::vector<TagDetection> extractTags(const cv::Mat& image);
	
};
typedef std::shared_ptr<TagDetector> TagDetectorPtr;
typedef std::shared_ptr<const TagDetector> TagDetectorConstPtr;

} // namespace

#endif
