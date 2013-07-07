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

#include "DirectionCalculation.h"
#include <ardrone_autonomy/Navdata.h>

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;
using namespace ardrone_autonomy;

  class LineDetection{

  private:
    vector<Vec4i> lines;
    DirectionCalculation direction_calc;
    DirectionArrow       direction;
    int altd;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber navdata_sub_;

  public:
    LineDetection() : it_(nh_)
    {
      image_pub_   = it_.advertise("/detected_lines_img", 1);
      image_sub_   = it_.subscribe("/ardrone/bottom/image_raw", 1, &LineDetection::ImageCallback, this);
      navdata_sub_ = nh_.subscribe("/ardrone/navdata",100,&LineDetection::NavDataCallback,this);
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    void NavDataCallback(const ardrone_autonomy::NavdataPtr& nav_msg){

      altd = nav_msg->altd;
    }
    void findLines(cv_bridge::CvImagePtr& cv_ptr){

      Mat image,image_edges;
      //cvtColor(cv_ptr->image,image,CV_BGR2HSV);
      inRange(cv_ptr->image,Scalar(200,200,200),Scalar(255,255,255),image);
      Canny(image, image_edges, 50, 200, 3);
      cvtColor(image_edges,cv_ptr->image, CV_GRAY2BGR);

      /* There are two possible hough methods, I used the probabilistic one */

       HoughLinesP(image_edges, lines, 1, CV_PI/180, 50, 50, 10 );
       ROS_INFO("Number of detected lines: %ld",lines.size());

       for( size_t i = 0; i < lines.size(); i++ )
       {
         Vec4i l = lines[i];
         line( cv_ptr->image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
       }

       // Calculate direction arrow
       if ((lines.size() > 0 )&& (lines.size() < 3)){
          direction_calc = DirectionCalculation(lines);
          direction_calc.publishErrors(Point2i(cv_ptr->image.cols/2,cv_ptr->image.rows/2),altd);
          direction = direction_calc.getDirection();
          // Draw direction line
          ROS_INFO("Direction: start = (%d,%d), end = (%d,%d)",direction.getStart().x,direction.getStart().y,direction.getEnd().x,direction.getEnd().y);
          line(cv_ptr->image,direction.getStart(),direction.getEnd(),Scalar(255,0,0),2,CV_AA);
      }
      image_pub_.publish(cv_ptr->toImageMsg());

    }
  };

#endif //LINEDETECTION_H
