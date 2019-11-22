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
  DetectTargetPtr detectTargetPtr(new DetectTarget(apriltag_yaml));
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
    detectTargetPtr->computeObservation(image);
    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key=='Q'){
      break;
    }
  }
  return 0;
}
