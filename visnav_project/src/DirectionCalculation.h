#ifndef DIRECTIONCALCULATION_H
#define DIRECTIONCALCULATION_H

#include "ros/ros.h"
#include "visnav_project/LineDetectionMsg.h"
#include <highgui.h>
#include <cv.h>

using namespace std;
using namespace cv;

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
    void findErrors(Point2i midPoint){

      // http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html

      double slope = (double)(midLine[3]-midLine[1])/(midLine[2]-midLine[0]);
      line_msg.errorYaw = atan(slope);

      // distance of mid-pixel to the line
      double distancetoLine = abs( (double)((midLine[2]-midLine[0]) * (midLine[1] - midPoint.y)) -
                                   ((midLine[0]-midPoint.x)*(midLine[3]-midLine[1]))
                                 )
                              /
                              sqrt((double)(midLine[2]-midLine[0])*(midLine[2]-midLine[0])+
                                   (midLine[3]-midLine[1])*(midLine[3]-midLine[1]));


    }
  };



#endif // DIRECTIONCALCULATION_H
