//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef KINECTDATAANALYZER_KINECTPARAMETERS_H
#define KINECTDATAANALYZER_KINECTPARAMETERS_H

#include <opencv2/opencv.hpp>

extern double depthToZ[2];  //from depth data(0~1) to Z in the camera coordinate(cm)
extern double projectionParameter[3][4];  //the projection matrix values

#endif //KINECTDATAANALYZER_KINECTPARAMETERS_H
