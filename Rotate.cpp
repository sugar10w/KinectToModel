#include<cmath>
#include<pcl/common/transforms.h>
#include"Rotate.h"

PointCloudPtr rotateAfterTime(PointCloudPtr cloud, float time)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  PointCloudPtr cloud_output(new PointCloud);
  float th=RotatingSpeed*time;

  transform(0,0)=cos(th);
  transform(0,2)=-sin(th);
  transform(2,0)=sin(th);
  transform(2,2)=cos(th);
  pcl::transformPointCloud(*cloud, *cloud_output, transform);
  
  return cloud_output;
}

