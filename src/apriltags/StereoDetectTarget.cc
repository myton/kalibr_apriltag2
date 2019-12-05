#include <target/StereoDetectTarget.h>
namespace target
{
  StereoDetectTarget::StereoDetectTarget(const std::string& aprilgrid_yaml, float minBorderDistance, bool showExtractionVideo, bool doSubpixRefinement, double maxSubpixDisplacement2):
  mDetectTargetPtr1(new DetectTarget(aprilgrid_yaml, minBorderDistance, showExtractionVideo, doSubpixRefinement, maxSubpixDisplacement2)), 
  mDetectTargetPtr2(new DetectTarget(aprilgrid_yaml, minBorderDistance, false, doSubpixRefinement, maxSubpixDisplacement2))
  {
  }
  
  bool StereoDetectTarget::computeObservation(const cv::Mat& left, const cv::Mat& right, cv::Mat& Points, cv::Mat& obervedPoints)
  {
    cv::Mat points1, obervedPoints1;
    cv::Mat points2, obervedPoints2;
    bool term1 = mDetectTargetPtr1->computeObservation(left, points1, obervedPoints1);
    bool term2 = mDetectTargetPtr2->computeObservation(right, points2, obervedPoints2);
    if (term1 && term2)
    {
      obervedPoints  = obervedPoints1 & obervedPoints2;
      Points = cv::Mat(points1.rows, points1.cols * 2, points1.type(), cv::Scalar(-1, -1));
      for (int i = 0; i < obervedPoints.rows; ++i)
      {
        if (obervedPoints.at<uchar>(i))
	{
	  Points.row(i).at<double>(0) = points1.row(i).at<double>(0);
	  Points.row(i).at<double>(1) = points1.row(i).at<double>(1);
	  Points.row(i).at<double>(2) = points2.row(i).at<double>(0);
	  Points.row(i).at<double>(3) = points2.row(i).at<double>(1);
	}
      }
      //std::cout <<"\n==============Points==============\n" <<  Points << std::endl;
      //std::cout <<"\n==============points1==============\n" <<  points1 << std::endl;
      //std::cout <<"\n==============points2==============\n" <<  points2 << std::endl;
      return true;
    }
    else
    {
      return false;
    }
  }
}
