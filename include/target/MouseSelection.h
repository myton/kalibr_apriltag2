#ifndef MOUSESELECTION_H
#define MOUSESELECTION_H
#include <opencv2/opencv.hpp>
#include <memory>
namespace target{
  class MouseSelection{
    public:
      explicit MouseSelection();
      void OnMouse(int event, int x, int y, int flgs);
      void setNearThre(int nearThre = 50);
      void showNearPoints(const cv::Mat& points, const cv::Mat& observedPoints, cv::Mat& img);
      std::vector<int> getIdx() const { return mvidx;}

    private:
      bool mshow;
      bool mselected;
      cv::Point2d mpt;
      int mnearThre; 
      std::vector<int> mvidx;
      //cv::Mat mpoints;
      //cv::Mat mobservePoints;
  };
  typedef std::shared_ptr<MouseSelection> MouseSelectionPtr;
}
#endif
