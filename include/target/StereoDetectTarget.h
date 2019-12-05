#ifndef STEREODETECTTARGET_H
#define STEREODETECTTARGET_H
#include <target/DetectTarget.h>
namespace target{
  class StereoDetectTarget{
    public: 
      StereoDetectTarget(const std::string& aprilgrid_yaml, float minBorderDistance = 4, bool showExtractionVideo = true, bool doSubpixRefinement = false, double maxSubpixDisplacement2 = 1.5); 
      bool computeObservation(const cv::Mat& left, const cv::Mat& right, cv::Mat& Points, cv::Mat& obervedPoints);
    private:
     DetectTargetPtr mDetectTargetPtr1; 
     DetectTargetPtr mDetectTargetPtr2; 
  };

  typedef std::shared_ptr<StereoDetectTarget> StereoDetectTargetPtr;
  typedef std::shared_ptr<const StereoDetectTarget> StereoDetectTargetConstPtr;
}
#endif
