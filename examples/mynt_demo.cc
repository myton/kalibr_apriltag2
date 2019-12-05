#include <iostream>
#include <target/DetectTarget.h>
#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <target/MouseSelection.h>
#include <target/StereoDetectTarget.h>

using namespace target;
int nearThre = 50;

void OnDepthMouseCallback(int event, int x, int y, int flags, void* data) {
  MouseSelection* mouseSelection = reinterpret_cast<MouseSelection*>(data);
  mouseSelection->OnMouse(event, x, y, flags);
}

void OnNearThe(int nearThre, void* data)
{
  MouseSelection* mouseSelection = reinterpret_cast<MouseSelection*>(data);
  mouseSelection->setNearThre(nearThre);
}

int main11(int argc, char* argv[]){
  if (argc != 2)
  {
    std::cout << "Uasge:" << std::string(argv[0]) <<  " apriltag_yaml"  << std::endl;
    return 1;
  }
  const std::string apriltag_yaml(argv[1]);
  std::cout << "apriltag_yaml: " << apriltag_yaml << std::endl;
  //DetectTarget(const std::string& aprilgrid_yaml, float minBorderDistance = 4, bool showExtractionVideo = true, bool doSubpixRefinement = false, double maxSubpixDisplacement2 = 1.5);
  StereoDetectTargetPtr stereoDetectTargetPtr(new StereoDetectTarget(apriltag_yaml, 4, true, false, 1.5)); 
  cv::VideoCapture cap;
  int camera_index = 6;
  while (!cap.open(camera_index)){
    if (--camera_index < 0)
    {
      std::cerr << "can not open device id " <<  ++camera_index << std::endl;
      return 1;
    }
  }
  
  cv::Mat image;
  cv::namedWindow( "image", 0 );
  MouseSelection mouseSelection;
  for (;;){
    cap >> image;
    if (image.type() != CV_8UC1)
    {
      cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }
    cv::Mat points, observPoints;
    if (stereoDetectTargetPtr->computeObservation(image, image, points, observPoints))
    {
      for (int i = 0; i < observPoints.rows; ++i)
      {
        if (observPoints.at<uchar>(i)){
          cv::circle(image, cv::Point2d(points.row(i).at<double>(0), points.row(i).at<double>(1)), 2, cv::Scalar(100), 1);
	}
      }
    }
    cv::imshow("image", image);
    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key=='Q'){
      break;
    }
  }
  return 0;
}

int main(int argc, char* argv[]){
  if (argc != 2)
  {
    std::cout << "Uasge:" << std::string(argv[0]) <<  " apriltag_yaml"  << std::endl;
    return 1;
  }
  const std::string apriltag_yaml(argv[1]);
  std::cout << "apriltag_yaml: " << apriltag_yaml << std::endl;
  //DetectTarget(const std::string& aprilgrid_yaml, float minBorderDistance = 4, bool showExtractionVideo = true, bool doSubpixRefinement = false, double maxSubpixDisplacement2 = 1.5);
  DetectTargetPtr detectTargetPtr(new DetectTarget(apriltag_yaml, 4, true, false, 1.5));
  cv::VideoCapture cap;
  int camera_index = 6;
  while (!cap.open(camera_index)){
    if (--camera_index < 0)
    {
      std::cerr << "can not open device id " <<  ++camera_index << std::endl;
      return 1;
    }
  }
  
  cv::Mat image;
  cv::namedWindow( "image", 0 );
  MouseSelection mouseSelection;
  for (;;){
    cap >> image;
    if (image.type() != CV_8UC1)
    {
      cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }

    cv::setMouseCallback("image",OnDepthMouseCallback, &mouseSelection);
    cv::createTrackbar( "nearThre", "image", &nearThre, std::min(image.rows, image.cols), &OnNearThe, &mouseSelection);
#if 1
    cv::Mat points, observPoints; 
    if (detectTargetPtr->computeObservation(image, points, observPoints)){
      mouseSelection.showNearPoints(points, observPoints, image);
      std::vector<int> vidx = mouseSelection.getIdx();
      std::cout << "vidx.sz: " << vidx.size() << std::endl;
    }
#endif
    cv::imshow("image", image);
    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key=='Q'){
      break;
    }
  }
  return 0;
}
