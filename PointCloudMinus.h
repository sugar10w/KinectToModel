
#ifndef _OBJECTFINDER_POINTCLOUDMINUS_
#define _OBJECTFINDER_POINTCLOUDMINUS_


#include "common.h"


PointCloudPtr kdTreeMinus(
    PointCloudPtr cloud_a, 
    PointCloudPtr cloud_b,  
    float threshold = 3,
    float distanceThreshold = 3
);

void removeOutlier(
  pcl::PointCloud<PointT>::Ptr cloud,
  pcl::PointCloud<PointT>::Ptr cloud_output,
  int K = 10,
  float distanceThreshold = 1
);


#endif //_OBJECTFINDER_POINTCLOUDMINUS_

