//
// Created by 郭嘉丞 on 15/9/12.
//

#ifndef KINECTDATAANALYZER_COMMON_H
#define KINECTDATAANALYZER_COMMON_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

#define __DEBUG__

#endif //KINECTDATAANALYZER_COMMON_H
