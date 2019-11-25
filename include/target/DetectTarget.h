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
      DetectTarget(const std::string& aprilgrid_yaml, float minBorderDistance = 4, bool showExtractionVideo = true, bool doSubpixRefinement = false, double maxSubpixDisplacement2 = 1.5);
      bool computeObservation(const cv::Mat& image, cv::Mat& Points, cv::Mat& obervedPoints);
    private:
      // size of apriltag, edge to edge [m]
      double mtagSize;
      //ratio of space between tags to tagSize
      double mtagSpacing; 
      //min. distance form image border for valid points [px]
      float mminBorderDistance;
      // show video during extraction
      bool mshowExtractionVideo;
      //subpixel refinement of extracted corners
      bool mdoSubpixRefinement;
      //max. displacement squarred in subpixel refinement  [px^2]
      double mmaxSubpixDisplacement2;
      int mtagRows;
      int mtagCols;

      AprilTags::TagCodesPtr mtagCodesPtr;
      AprilTags::TagDetectorPtr mtagDetector;
  };
  typedef std::shared_ptr<DetectTarget> DetectTargetPtr;
  typedef std::shared_ptr<const DetectTarget> DetectTargetConstPtr;
} 
#endif
