#include <iostream>
#include <target/DetectTarget.h>
#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>

using namespace target;
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
  for (;;){
    cap >> image;
    if (image.type() != CV_8UC1)
    {
      cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }
    cv::Mat points, observPoints; 
    if (detectTargetPtr->computeObservation(image, points, observPoints)){
      for (int i = 0; i < observPoints.rows; ++i){
        std::cout << i << ": "; 
        if (observPoints.at<uchar>(i, 0)){
          std::cout << "{" << points.row(i) << "}, \t";
          assert(points.row(i).at<double>(0) > 0 && points.row(i).at<double>(0) < image.cols);
          assert(points.row(i).at<double>(1) > 0 && points.row(i).at<double>(1) < image.rows);
        }else
        {
          std::cout << "\n**************************\n";
        }
      }
    }
    std::cout << "\n===================\n";

    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key=='Q'){
      break;
    }
  }
  return 0;
}
