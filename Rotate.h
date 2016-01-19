
#ifndef _OBJECTSCAN_ROTATE_
#define _OBJECTSCAN_ROTATE_

#include"common.h"

const float RotatingSpeed =  6.28 / 62.5;  // (roll plane, clockwise)

PointCloudPtr rotateAfterTime(PointCloudPtr cloud, float time);

#endif //_OBJECTSCAN_ROTATE_

