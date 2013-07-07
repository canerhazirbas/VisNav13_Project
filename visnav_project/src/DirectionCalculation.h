#ifndef DIRECTIONCALCULATION_H
#define DIRECTIONCALCULATION_H

#include "ros/ros.h"
#include "visnav_project/LineDetectionMsg.h"
#include <highgui.h>
#include <cv.h>

using namespace std;
using namespace cv;

#define FOCAL_LENGTH 700.490828918144
#define EPSILON      0.0000001

class DirectionArrow{

  private:
    Point2i start_;
    Point2i end_;

  public:

    DirectionArrow(){}
    DirectionArrow(Vec4i v){
      if(v[1] < v[3])
        {
          start_ = Point2i(v[2],v[3]);
          end_   = Point2i(v[0],v[1]);
        }
      else
        {
          start_ = Point2i(v[0],v[1]);
          end_   = Point2i(v[2],v[3]);
        }

    }

    Point2i getStart(){
      return start_;
    }
    Point2i getEnd(){
      return end_;
    }


  };

  class DirectionCalculation{

  private:
    DirectionArrow direction;
    vector<Vec4i> lines;
    Vec4i midLine;
    visnav_project::LineDetectionMsg line_msg;

  public:

    DirectionCalculation(){}
    DirectionCalculation(vector<Vec4i> lines)
    {
      if(lines.size()==1)
        direction = DirectionArrow(lines[0]);

      else if (lines.size() ==2){

          DirectionArrow first(lines[0]);
          DirectionArrow second(lines[1]);
          midLine[0] = (first.getStart().x + second.getStart().x) / 2;
          midLine[1] = (first.getStart().y + second.getStart().y) / 2;
          midLine[2] = (first.getEnd().x + second.getEnd().x) / 2;
          midLine[3] = (first.getEnd().y + second.getEnd().y) / 2;
          direction = DirectionArrow(midLine);
          // 0-1 start, 2-3 end x,y
      }
      else
        // Dont know yet in case we have multi-detected(>2) lines
        return;
    }
    DirectionArrow getDirection(){
      return direction;
    }
    void publishErrors(Point2i midPoint,int altd){

      // http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
      Point2i start_ = direction.getStart();
      Point2i end_   = direction.getEnd();

      if ((end_.x - start_.x) != 0){
         double slope = (double)((end_.y - start_.y) / (end_.x - start_.x));
         line_msg.error_yaw = atan(slope);
      }
      else
        line_msg.error_yaw = 1.571 ; // 90 degrees in radian


      ROS_INFO("Angle of line : %.2f",line_msg.error_yaw);
      // distance of mid-pixel to the line
      double distancetoLine = abs( (double)((end_.x - start_.x) * (start_.y - midPoint.y)) -
                                   ((start_.x-midPoint.x)*(end_.y-start_.y))
                                 )
                              /
                              sqrt((double)(end_.x-start_.x)*(end_.x-start_.x)+
                                   (end_.y-start_.y)*(end_.y-start_.y));

      // global distance calculation x = d*Z / f; FOCAL_LENGTH normalized by size of image
      double global_dist_to_line = distancetoLine * altd / (FOCAL_LENGTH * (midPoint.x *2 ));
      ROS_INFO("Distance of midPoint to line : %.2f",distancetoLine);
      ROS_INFO("Global Distance of midPoint to line : %.2f",global_dist_to_line);
    }
  };



#endif // DIRECTIONCALCULATION_H
