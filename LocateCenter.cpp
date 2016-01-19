// sugar10w, 2016.1.18

#include<vector>
#include<pcl/filters/passthrough.h>
#include"LocateCenter.h"


float centerAverage(std::vector<float> & vec, float centerPart = 0.6f)
{
  std::sort(vec.begin(), vec.end());
  float sum=0;
  int st=(0.5f-centerPart/2)*vec.size(), ed=(0.5f+centerPart/2)*vec.size();
  if (st>=ed) return 0;
  for (int i=st; i<ed; ++i) sum+=vec[i];
  return sum/(ed-st);
}

PointT locateCenterPoint(PointCloudPtr cloud)
{
  int size = cloud->width * cloud->height;
  PointT weightPoint;
  
  std::vector<float> vX, vY;
  float minZ=10000;
  
  PointCloudPtr cloud_1(new PointCloud),
                cloud_2(new PointCloud);

  for (int i=0; i<size; ++i)
  { 
    PointT point = cloud->points[i];
    vX.push_back(point.x);
    vY.push_back(point.y);
  } 
  weightPoint.x=centerAverage(vX);
  weightPoint.y=centerAverage(vY);
  
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(weightPoint.x-4.f, weightPoint.x+4.f);
  pass.filter(*cloud_1);
  
  pass.setInputCloud(cloud_1);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(weightPoint.y-1.9f, weightPoint.y+1.9f);
  pass.filter(*cloud_2);
  
  
  for (int i=0; i<cloud_2->width*cloud_2->height; ++i)
    if (cloud_2->points[i].z < minZ) minZ = cloud_2->points[i].z;
  weightPoint.z = minZ+RadiusZ;

  return weightPoint;
}


void movePointCloud(PointCloudPtr cloud, float x, float y, float z)
 {
  int size = cloud->width * cloud->height;
  for (int i=0; i<size; ++i)
   {
    PointT& point = cloud->points[i];
    point.x+=x;
    point.y+=y;
    point.z+=z;
   }
}

void zoomPointCloud(PointCloudPtr cloud, float x, float y, float z)
{
  int size = cloud->width * cloud->height;
  for (int i=0; i<size; ++i)
  {
    PointT& point = cloud->points[i];
    point.x*=x;
    point.y*=y;
    point.z*=z;
  }
}

PointCloudPtr cutNearCenter(PointCloudPtr cloud, PointT center)
{
  PointCloudPtr cloud_1(new PointCloud), cloud_2(new PointCloud), cloud_3(new PointCloud);
  pcl::PassThrough<PointT> pass;
  
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(center.x-maxWidth/2, center.x+maxWidth/2);
  pass.filter(*cloud_1);
  
  pass.setInputCloud(cloud_1);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(center.y, center.y+maxHeight+2);
  pass.filter(*cloud_2);
  
  pass.setInputCloud(cloud_2);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(center.z-maxWidth/2, center.z+plusDepth);
  pass.filter(*cloud_3);
  
  movePointCloud(cloud_3, -center.x, -center.y, -center.z);
  zoomPointCloud(cloud_3, 1, 1, zZoom);
  return cloud_3;
}
