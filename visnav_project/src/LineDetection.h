#ifndef LINEDETECTION_H
#define LINEDETECTION_H

#include "ros/ros.h"
#include <highgui.h>
#include <cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

namespace visnav_project{

  class LineDetection{

  private:
    Mat image;
    vector<Vec4i> lines;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  public:
    LineDetection() : it_(nh_)
    {
      image_pub_ = it_.advertise("/detected_lines_img", 1);
      image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, &LineDetection::readImage, this);

    }

    void readImage(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      findLines(cv_ptr);
    }
    void findLines(cv_bridge::CvImagePtr& cv_ptr){

      Mat image,image_edges;
      //cvtColor(cv_ptr->image,image,CV_BGR2HSV);
      inRange(cv_ptr->image,Scalar(200,200,200),Scalar(255,255,255),image);
      Canny(image, image_edges, 50, 200, 3);
      cvtColor(image_edges,cv_ptr->image, CV_GRAY2BGR);

      /* There are two possible hough methods, I used the probabilistic one */

       HoughLinesP(image_edges, lines, 1, CV_PI/180, 50, 50, 10 );
       for( size_t i = 0; i < lines.size(); i++ )
       {
         Vec4i l = lines[i];
         line( cv_ptr->image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
       }
      image_pub_.publish(cv_ptr->toImageMsg());
      ROS_INFO("Number of detected lines: %ld",lines.size());
    }
  };
}

#endif //LINEDETECTION_H
