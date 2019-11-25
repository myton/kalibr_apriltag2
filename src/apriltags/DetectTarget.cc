#include <target/DetectTarget.h>
#include <cassert>
namespace target{
  DetectTarget::DetectTarget(const std::string& apriltag_yaml, float minBorderDistance, bool showExtractionVideo, bool doSubpixRefinement, double maxSubpixDisplacement2):mminBorderDistance(minBorderDistance), mshowExtractionVideo(showExtractionVideo), mdoSubpixRefinement(doSubpixRefinement), mmaxSubpixDisplacement2(maxSubpixDisplacement2){
    cv::FileStorage fs(apriltag_yaml, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      std::cerr << "can not open " << apriltag_yaml << " file!" << std::endl;
      assert(false);
    }
    std::string target_type;
    fs["target_type"] >> target_type;
    fs["tagRows"] >> mtagRows;
    fs["tagCols"] >> mtagCols;
    fs["tagSize"] >> mtagSize;
    fs["tagSpacing"] >> mtagSpacing;
    std::cout << "=========config ==================" << std::endl;
    std::cout << "--target_type: " << target_type << std::endl;
    std::cout << "--tagRows: "  << mtagRows << std::endl;
    std::cout << "--tagCols: " << mtagCols << std::endl;
    std::cout << "--tagSize: " << mtagSize << std::endl;
    std::cout << "--tagSpacing: " << mtagSpacing << std::endl;
    assert(target_type == "36h11");
    mtagCodesPtr.reset(new AprilTags::TagCodes(AprilTags::tagCodes36h11));
    mtagDetector.reset(new AprilTags::TagDetector(*mtagCodesPtr, 2));

    if (mshowExtractionVideo) {
      cv::namedWindow("Aprilgrid: Tag detection");
      cv::namedWindow("Aprilgrid: Tag corners");
      cvStartWindowThread();
    }
  }

  bool DetectTarget::computeObservation(const cv::Mat& image, cv::Mat& points, cv::Mat& obervedPoints){
    CV_Assert(image.depth() == CV_8UC1 && image.type() == CV_8UC1);
    points = cv::Mat(mtagRows * mtagCols * 4, 2, CV_64F, cv::Scalar(-1, -1));
    obervedPoints = cv::Mat(mtagRows * mtagCols * 4, 1, CV_8UC1, cv::Scalar(0));

    bool success = true;
    std::vector<AprilTags::TagDetection> detections = mtagDetector->extractTags(image);
    //min. distance [px] of tag corners from image border (tag is not used if violated)
    std::vector<AprilTags::TagDetection>::iterator iter = detections.begin();
    for (iter = detections.begin(); iter != detections.end();) {
      // check all four corners for violation
      bool remove = false;

      for (int j = 0; j < 4; j++) {
        remove |= iter->p[j].first < mminBorderDistance;
        remove |= iter->p[j].first > (float) (image.cols) - mminBorderDistance;  //width
        remove |= iter->p[j].second < mminBorderDistance;
        remove |= iter->p[j].second > (float) (image.rows) - mminBorderDistance;  //height
      }

      //also remove tags that are flagged as bad
      if (iter->good != 1)
        remove |= true;

      //also remove if the tag ID is out-of-range for this grid (faulty detection)
      // tagRows * tagCols
      if (iter->id >= (int) mtagRows * mtagRows * 4)
        remove |= true;

      // delete flagged tags
      if (remove) {
        std::cerr << "Tag with ID " << iter->id << " is only partially in image (corners outside) and will be removed from the TargetObservation.\n";

        // delete the tag and advance in list
        iter = detections.erase(iter);
      } else {
        //advance in list
        ++iter;
      }
    } //for

    //did we find enough tags?
    int minTagsForValidObs = std::min(mtagRows * 2, mtagCols*2);  
    if (detections.size() < minTagsForValidObs) {
      success = false;
    }


    //sort detections by tagId
    std::sort(detections.begin(), detections.end(), AprilTags::TagDetection::sortByIdCompare);

    // check for duplicate tagIds (--> if found: wild Apriltags in image not belonging to calibration target)
    // (only if we have more than 1 tag...)
    if (detections.size() > 1) {
      for (unsigned i = 0; i < detections.size() - 1; i++)
        if (detections[i].id == detections[i + 1].id) {
          //show the duplicate tags in the image
          cv::destroyAllWindows();
          cv::namedWindow("Wild Apriltag detected. Hide them!");
          cvStartWindowThread();

          cv::Mat imageCopy = image.clone();
          cv::cvtColor(imageCopy, imageCopy, CV_GRAY2RGB);

          //mark all duplicate tags in image
          for (int j = 0; i < detections.size() - 1; i++) {
            if (detections[j].id == detections[j + 1].id) {
              detections[j].draw(imageCopy);
              detections[j + 1].draw(imageCopy);
            }
          }

          cv::putText(imageCopy, "Duplicate Apriltags detected. Hide them.",
                      cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                      CV_RGB(255,0,0), 2, 8, false);
          cv::putText(imageCopy, "Press enter to exit...", cv::Point(50, 80),
                      CV_FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255,0,0), 2, 8, false);
          cv::imshow("Duplicate Apriltags detected. Hide them", imageCopy);  // OpenCV call

          // and exit
          std::cerr << "\n[ERROR]: Found apriltag not belonging to calibration board. Check the image for the tag and hide it.\n";

          cv::waitKey();
          exit(0);
        }
    } //if

    // convert corners to cv::Mat (4 consecutive corners form one tag)
    /// point ordering here
    ///          11-----10  15-----14
    ///          | TAG 2 |  | TAG 3 |
    ///          8-------9  12-----13
    ///          3-------2  7-------6
    ///    y     | TAG 0 |  | TAG 1 |
    ///   ^      0-------1  4-------5
    ///   |-->x
    cv::Mat tagCorners(4 * detections.size(), 2, CV_32F);

    for (unsigned i = 0; i < detections.size(); i++) {
      for (unsigned j = 0; j < 4; j++) {
        tagCorners.at<float>(4 * i + j, 0) = detections[i].p[j].first;
        tagCorners.at<float>(4 * i + j, 1) = detections[i].p[j].second;
      }
    }

    //store a copy of the corner list before subpix refinement
    cv::Mat tagCornersRaw = tagCorners.clone();

    //optional subpixel refinement on all tag corners (four corners each tag)
    if (mdoSubpixRefinement && success)
      cv::cornerSubPix(
          image, tagCorners, cv::Size(2, 2), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	
    if (mshowExtractionVideo) {
      //image with refined (blue) and raw corners (red)
      cv::Mat imageCopy1 = image.clone();
      cv::cvtColor(imageCopy1, imageCopy1, CV_GRAY2RGB);
#if 0
      for (unsigned i = 0; i < detections.size(); i++)
        for (unsigned j = 0; j < 4; j++) {
          //raw apriltag corners
          //cv::circle(imageCopy1, cv::Point2f(detections[i].p[j].first, detections[i].p[j].second), 2, CV_RGB(255,0,0), 1);

          //subpixel refined corners
          cv::circle(
              imageCopy1,
              cv::Point2f(tagCorners.at<float>(4 * i + j, 0),
                          tagCorners.at<float>(4 * i + j, 1)),
              3, CV_RGB(0,0,255), 1);

          if (!success)
            cv::putText(imageCopy1, "Detection failed! (frame not used)",
                        cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                        CV_RGB(255,0,0), 3, 8, false);
        }
#else
      const int _cols = mtagCols* 2;
      for (unsigned i = 0; i < detections.size(); i++)
      {
        // get the tag id
        unsigned int tagId = detections[i].id;

        // calculate the grid idx for all four tag corners given the tagId and cols
        unsigned int baseId = (int) (tagId / (_cols / 2)) * _cols * 2
            + (tagId % (_cols / 2)) * 2;
        unsigned int pIdx[] = { baseId, baseId + 1, baseId + (unsigned int) _cols
            + 1, baseId + (unsigned int) _cols };
	  
        for (unsigned j = 0; j < 4; j++) {
          //raw apriltag corners
          //cv::circle(imageCopy1, cv::Point2f(detections[i].p[j].first, detections[i].p[j].second), 2, CV_RGB(255,0,0), 1);

          //subpixel refined corners
          cv::circle(
              imageCopy1,
              cv::Point2f(tagCorners.at<float>(4 * i + j, 0),
                          tagCorners.at<float>(4 * i + j, 1)),
              3, CV_RGB(0,0,255), 1);
	  cv::putText(imageCopy1, std::to_string(pIdx[j]),
              cv::Point2f(tagCorners.at<float>(4 * i + j, 0),
                          tagCorners.at<float>(4 * i + j, 1)),
	     CV_FONT_HERSHEY_SIMPLEX, 0.3,
             CV_RGB(0,0,255)
	  );

          if (!success)
            cv::putText(imageCopy1, "Detection failed! (frame not used)",
                        cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                        CV_RGB(255,0,0), 3, 8, false);
        }
      }

#endif

      cv::imshow("Aprilgrid: Tag corners", imageCopy1);  // OpenCV call
      cv::waitKey(1);

      /* copy image for modification */
      cv::Mat imageCopy2 = image.clone();
      cv::cvtColor(imageCopy2, imageCopy2, CV_GRAY2RGB);
      /* highlight detected tags in image */
      for (unsigned i = 0; i < detections.size(); i++) {
        detections[i].draw(imageCopy2);

        if (!success)
          cv::putText(imageCopy2, "Detection failed! (frame not used)",
                      cv::Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.8,
                      CV_RGB(255,0,0), 3, 8, false);
      }

      cv::imshow("Aprilgrid: Tag detection", imageCopy2);  // OpenCV call
      cv::waitKey(1);

      //if success is false exit here (delayed exit if _options.showExtractionVideo=true for debugging)
      if (!success)
        return success;
    }//if

  //insert the observed points into the correct location of the grid point array
  /// point ordering
  ///          12-----13  14-----15
  ///          | TAG 2 |  | TAG 3 |
  ///          8-------9  10-----11
  ///          4-------5  6-------7
  ///    y     | TAG 0 |  | TAG 1 |
  ///   ^      0-------1  2-------3
  ///   |-->x

  //outCornerObserved.resize(size(), false);
  //outImagePoints.resize(size(), 2);
  const int _cols = mtagCols* 2;
  for (unsigned int i = 0; i < detections.size(); i++) {
    // get the tag id
    unsigned int tagId = detections[i].id;

    // calculate the grid idx for all four tag corners given the tagId and cols
    unsigned int baseId = (int) (tagId / (_cols / 2)) * _cols * 2
        + (tagId % (_cols / 2)) * 2;
    unsigned int pIdx[] = { baseId, baseId + 1, baseId + (unsigned int) _cols
        + 1, baseId + (unsigned int) _cols };

    // add four points per tag
    for (int j = 0; j < 4; j++) {
      //refined corners
      double corner_x = tagCorners.row(4 * i + j).at<float>(0);
      double corner_y = tagCorners.row(4 * i + j).at<float>(1);

      //raw corners
      double cornerRaw_x = tagCornersRaw.row(4 * i + j).at<float>(0);
      double cornerRaw_y = tagCornersRaw.row(4 * i + j).at<float>(1);

      //only add point if the displacement in the subpixel refinement is below a given threshold
      double subpix_displacement_squarred = (corner_x - cornerRaw_x)
          * (corner_x - cornerRaw_x)
          + (corner_y - cornerRaw_y) * (corner_y - cornerRaw_y);

      //add all points, but only set active if the point has not moved to far in the subpix refinement
//      outImagePoints.row(pIdx[j]) = Eigen::Matrix<double, 1, 2>(corner_x,
//                                                                corner_y);
       points.row(pIdx[j]).at<double>(0) = corner_x;
       points.row(pIdx[j]).at<double>(1) = corner_y;

      if (subpix_displacement_squarred <= mmaxSubpixDisplacement2) {
        obervedPoints.row(pIdx[j]) = 1;
      } else {
        std::cerr << "Subpix refinement failed for point: " << pIdx[j] << " with displacement: " << sqrt(subpix_displacement_squarred) << "(point removed) \n";
        obervedPoints.row(pIdx[j]) = 0;
      }
    }
  }//for
    return success;
  } //function

} //namespace
