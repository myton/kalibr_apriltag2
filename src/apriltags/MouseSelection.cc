#include <target/MouseSelection.h>
namespace target{
  MouseSelection::MouseSelection():mshow(false), mselected(false), mpt(cv::Point(0, 0)), mnearThre(50)
  {
  }

  void MouseSelection::OnMouse(int event, int x, int y, int flgs){
  
    if (event != cv::EVENT_MOUSEMOVE && event != cv::EVENT_LBUTTONDOWN) {
      return;
    }
    mshow = true;

    if (event == cv::EVENT_MOUSEMOVE) {
      if (!mselected) {
        mpt.x = x;
        mpt.y = y;
      }
    } else if (event == cv::EVENT_LBUTTONDOWN) {
      if (mselected) {
          mselected = false;
      } else {
        mselected = true;
      }
      mpt.x = x;
      mpt.y = y;
    }

//    std::cout << mpt << std::endl;
  }

  void MouseSelection::setNearThre(int nearThre)
  {
    mnearThre = nearThre;
  }

  void MouseSelection::showNearPoints(const cv::Mat& points, const cv::Mat& observedPoints, cv::Mat& img)
  {
    std::vector<int> vidx;
    vidx.reserve(points.total() * 0.5);
    for (int r = 0; r < observedPoints.rows; ++r)
    {
      if (observedPoints.at<uchar>(r, 0))
      {
        cv::Point2d pt(points.row(r).at<double>(0), points.row(r).at<double>(1));
	double distance = std::sqrt(std::pow(mpt.x-pt.x, 2) + std::pow(mpt.y - pt.y, 2));
	//std::cout << "distance: " << distance << ", "<< pt << ", " << mpt <<  std::endl;
	if (distance < mnearThre)
	{
	  vidx.push_back(r);
	}
      }
    }
    for (int i = 0; i < vidx.size(); ++i)
    {
      cv::Mat ri = points.row(vidx[i]);
      cv::circle(img, cv::Point2d(ri.at<double>(0), ri.at<double>(1)), 1, cv::Scalar(100), 2);
    }

    mvidx.swap(vidx);
  }
}
