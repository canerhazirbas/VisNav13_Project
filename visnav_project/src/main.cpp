#include <stdio.h>
#include "LineDetection.h"

int main(int argc, char** argv ){

  ros::init(argc, argv, "visnav_project");
  LineDetection line_detection;
  ros::spin();
  return 0;
}
