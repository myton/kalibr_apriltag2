#ifndef DETECTTARGET
#define DETECTTARGET 
#include <opencv2/opencv.hpp>
#include <memory>
#include <target/DetectTarget.h>
#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>

namespace target{
  class DetectTarget{
    public:
      DetectTarget(const std::string& aprilgrid_yaml, float minBorderDistance = 4, bool showExtractionVideo = true, bool doSubpixRefinement = false);
      bool computeObservation(const cv::Mat& image);
    private:
      // size of apriltag, edge to edge [m]
      double mtagSize;
      //ratio of space between tags to tagSize
      double mtagSpacing; 
      float mminBorderDistance;
      bool mshowExtractionVideo;
      bool mdoSubpixRefinement;

      cv::Mat mPoints;
      AprilTags::TagCodesPtr mtagCodesPtr;
      AprilTags::TagDetectorPtr mtagDetector;
  };
  typedef std::shared_ptr<DetectTarget> DetectTargetPtr;
  typedef std::shared_ptr<const DetectTarget> DetectTargetConstPtr;
} 
#endif
