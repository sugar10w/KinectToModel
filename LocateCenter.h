// sugar10w, 2016.1.19

#ifndef _OBJECTBUILDER_LOCATECENTER_
#define _OBJECTBUILDER_LOCATECENTER_

#include"common.h"

const float zZoom = 0.5f;
const float RadiusZ = 4.0f/zZoom;
const float maxWidth = 25.0f, maxHeight = 30.0f, plusDepth = 1.0f;

PointT locateCenterPoint(PointCloudPtr cloud);

PointCloudPtr cutNearCenter(PointCloudPtr cloud, PointT center);

#endif //_OBJECTBUILDER_LOCATECENTER_
