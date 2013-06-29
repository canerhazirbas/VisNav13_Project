#ifndef DIRECTIONCALCULATION_H
#define DIRECTIONCALCULATION_H

#include "ros/ros.h"
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

  public:

    DirectionCalculation(){}
    DirectionCalculation(vector<Vec4i> lines)
    {
      if(lines.size()==1)
        direction = DirectionArrow(lines[0]);

      else if (lines.size() ==2){

          DirectionArrow first(lines[0]);
          DirectionArrow second(lines[1]);
          Vec4i midLine;
          midLine[0] = (first.getStart().x + second.getStart().x) / 2;
          midLine[1] = (first.getStart().y + second.getStart().y) / 2;
          midLine[2] = (first.getEnd().x + second.getEnd().x) / 2;
          midLine[3] = (first.getEnd().y + second.getEnd().y) / 2;
          direction = DirectionArrow(midLine);

      }
      //else
        // Dont know yet in case we have multi-detected lines

    }
    DirectionArrow getDirection(){
      return direction;
    }
  };



#endif // DIRECTIONCALCULATION_H
