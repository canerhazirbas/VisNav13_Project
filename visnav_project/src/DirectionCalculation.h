#ifndef DIRECTIONCALCULATION_H
#define DIRECTIONCALCULATION_H

#include "ros/ros.h"
#include "visnav_project/LineDetectionMsg.h"
#include <highgui.h>
#include <cv.h>
#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace cv;

#define FOCAL_LENGTH 700.490828918144
#define LEFT          1
#define RIGHT        -1

 class DirectedLine{

  private:
    Point2i start_;
    Point2i end_;

  public:

    DirectedLine(){}
    DirectedLine(Vec4i v){
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
    DirectedLine direction;
    vector<Vec4i> lines;

    visnav_project::LineDetectionMsg line_msg;

  public:

    DirectionCalculation(){}
    DirectionCalculation(vector<Vec4i> lines)
    {
      Vec4i midLine(1);
      vector<DirectedLine> directed_lines(lines.size());

      for(unsigned int i = 0; i < lines.size(); i++)
      {
          directed_lines[i] =DirectedLine(lines[i]);
          midLine[0] += directed_lines[i].getStart().x;
          midLine[1] += directed_lines[i].getStart().y;
          midLine[2] += directed_lines[i].getEnd().x;
          midLine[3] += directed_lines[i].getEnd().y;
      }
      midLine[0] /= lines.size();
      midLine[1] /= lines.size();
      midLine[2] /= lines.size();
      midLine[3] /= lines.size();
      direction = DirectedLine(midLine);
    }
    DirectedLine getDirection(){
      return direction;
    }
    visnav_project::LineDetectionMsg calcErrors(Point2i midPoint,int altd){

      // http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
      Point2i start_ = direction.getStart();
      Point2i end_   = direction.getEnd();
      double slope;

      if ((double)(end_.y - start_.y) != 0){
         slope = ((double)(end_.x - start_.x) / (double)(end_.y - start_.y));
         line_msg.error_yaw = atan(slope);
      }
      else
        line_msg.error_yaw = 0	 ; // 90 degrees in radian

      // distance of mid-pixel to the line
      double distancetoLine = abs( (double)((end_.x - start_.x) * (start_.y - midPoint.y)) -
                                   ((start_.x-midPoint.x)*(end_.y-start_.y))
                                 )
                              /
                              sqrt((double)(end_.x-start_.x)*(end_.x-start_.x)+
                                   (end_.y-start_.y)*(end_.y-start_.y));

      // global distance calculation x = d*Z / f; //FOCAL_LENGTH normalized by size of image
      line_msg.error_pitch = (double)(distancetoLine * (altd*0.001) / FOCAL_LENGTH) * (midPoint.x *2) * 0.001;
      ROS_INFO("Slope_X : %.2f ; Distance_L : %.2f pixel; Distance_G : %.4f meters",line_msg.error_yaw,distancetoLine,line_msg.error_pitch);
      return line_msg;
    }
    // Find if midPoint lies on left-right of line
/*    int findDistanceDirection(DirectedLine Dl, Point2i midPoint){

      Point2i start_ = Dl.getStart();
      Point2i end_   = Dl.getEnd();

      if((midPoint.x < start_.x) && ((midPoint.x < end_.x)))
          return  LEFT;

      if((midPoint.x > start_.x) && ((midPoint.x > end_.x)))
          return  RIGHT;

      else if((midPoint.x < start_.x) && ((midPoint.x > end_.x)))
        if(midPoint.y < end_.y)
            return LEFT;
        else
            return RIGHT;

    }
*/
  };



#endif // DIRECTIONCALCULATION_H
