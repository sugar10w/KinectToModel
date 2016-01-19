#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>
#include "PointCloudMinus.h"

float average(
   std::vector<float>& vec
 )
{
  float sum;
  for (int i=0; i<vec.size(); ++i) sum+=vec[i];
  return sum/vec.size();
}

float averageKDistance(
   pcl::KdTreeFLANN<PointT> & kdtree,
   std::vector<int> & pointIdxNKNSearch,
   std::vector<float> & pointNKNSquaredDistance,
   PointT point,
   int K 
 )
{
 
  if (kdtree.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
   {
    return average(pointNKNSquaredDistance);
   }
  else
    {
    return -1;
  }

} 


PointCloudPtr kdTreeMinus(
    PointCloudPtr cloud_a, 
    PointCloudPtr cloud_b,
    float threshold,
    float distanceThreshold
)
{
 std::vector<int> chosenIndices;
 int size = cloud_a->width * cloud_a->height;

 pcl::KdTreeFLANN<PointT> kdtree_a, kdtree_b;
 kdtree_a.setInputCloud(cloud_a);
 kdtree_b.setInputCloud(cloud_b);
 
 int K = 10;
 std::vector<int> pointIdxNKNSearch(K);
 std::vector<float> pointNKNSquaredDistance(K);
 
 for (int i=0; i<size; ++i)
 {
  PointT point = cloud_a->points[i];
  float avgKDis_a = averageKDistance(kdtree_a, pointIdxNKNSearch, pointNKNSquaredDistance, point, K),
        avgKDis_b = averageKDistance(kdtree_b, pointIdxNKNSearch, pointNKNSquaredDistance, point, K);
  if (avgKDis_a<distanceThreshold && abs(avgKDis_a-avgKDis_b)>threshold) chosenIndices.push_back(i);    
 } 
  
  PointCloudPtr cloud_output(new PointCloud);
  cloud_output->width = chosenIndices.size();
  cloud_output->height = 1;
  cloud_output->resize(chosenIndices.size());

  for (int i=0; i<chosenIndices.size(); ++i) cloud_output->points[i]=cloud_a->points[chosenIndices[i]];
  
  return cloud_output;
} 

void removeOutlier(
  PointCloudPtr cloud,
  PointCloudPtr cloud_output,
  int K,
  float distanceThreshold
)
{
  int size = cloud->width * cloud->height;

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud);
  
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  std::vector<int> chosenIndices;

  for (int i=0; i<size; ++i)
  {
    PointT point = cloud->points[i];
    float avgKDis = averageKDistance(kdtree, pointIdxNKNSearch, pointNKNSquaredDistance, point, K);
    if (avgKDis<distanceThreshold) chosenIndices.push_back(i);    
  } 
  
  cloud_output->width = chosenIndices.size();
  cloud_output->height = 1;
  cloud_output->resize(chosenIndices.size());
  for (int i=0; i<chosenIndices.size(); ++i) cloud_output->points[i]=cloud->points[chosenIndices[i]];
 }


