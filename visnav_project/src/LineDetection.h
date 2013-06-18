#include "ros/ros.h"
#include <highgui.h>
#include <cv.h>

using namespace cv;

class LineDetection{

private:
  Mat image;
  vector<Vec4i> lines;

public:
  LineDetection()
  {
    readImage();
    findLines();
  }

  void readImage(){

    image = imread("/home/caner/fuerte_workspace/VisNav13_Project/visnav_project/src/image.jpg");
    if (image.empty())
      {
        ROS_ERROR("Image is not found");
        exit(-1);
      }
  }

  void findLines(){

    Mat image_edges,image_lines;

    Canny(image, image_edges, 50, 200, 3);
    cvtColor(image_edges,image_lines, CV_GRAY2BGR);

    /* There are two possible hough methods, I used the probabilistic one for now from sample code */

     HoughLinesP(image_edges, lines, 1, CV_PI/180, 50, 50, 10 );
     for( size_t i = 0; i < lines.size(); i++ )
     {
       Vec4i l = lines[i];
       line( image_lines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
     }

    imshow("source", image);
    imshow("detected lines", image_lines);

    waitKey();

  }
};

