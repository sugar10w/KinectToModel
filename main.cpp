
#include <cstdio>
#include <string>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include "common.h"
#include "LoadRGBD.h"
#include "PointCloudMinus.h"
#include "LocateCenter.h"
#include "Rotate.h"

int main(int argc, const char * argv[])
{
  PointCloudPtr cloud_1 = loadRGBD2Cloud(-1);
  PointCloudPtr cloud_0 = loadRGBD2Cloud(0);
  PointCloudPtr rollingPlane = kdTreeMinus(cloud_0, cloud_1);
  PointT rollingPoint = locateCenterPoint(rollingPlane);
  PointCloudPtr cloud_minus = cutNearCenter( cloud_0, rollingPoint );
  PointCloudPtr cloud_sum(new PointCloud);
  int frames;
  float time;
  
  pcl::io::savePCDFile("roll.pcd", *cloud_0);
  
  std::ifstream info_file("kinect_info.txt");
  info_file >> frames;

  for (int i=1; i<=frames; ++i)
  {
    char buffer[20]; sprintf(buffer, "%d", i); std::string prefix = "res", prefix2 = "raw";
    PointCloudPtr res =  kdTreeMinus( cutNearCenter( loadRGBD2Cloud(i) , rollingPoint), cloud_minus ) ;
    pcl::io::savePCDFile(prefix+buffer+".pcd", *res);
    
    info_file >> time;
    *cloud_sum += *(rotateAfterTime(res, time));
  }

  pcl::io::savePCDFile("summary.pcd", *cloud_sum);

  return 0;
}


