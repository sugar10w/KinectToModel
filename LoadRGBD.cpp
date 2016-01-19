#include<cstdio>
#include<opencv2/opencv.hpp>
#include<pcl/io/pcd_io.h>
#include"PointCloudBuilder.h"
#include"LoadRGBD.h"

using namespace std;


cv::Mat reload_32f_image(string filename)
{
    ifstream fin(filename.c_str(), ifstream::binary);
    int num_rows, num_cols;
    fin.read((char *) &num_rows, sizeof(int));
    fin.read((char *) &num_cols, sizeof(int));
    cv::Mat mat = cv::Mat::zeros(num_rows, num_cols, CV_32FC1);
    fin.read((char *) mat.data, num_cols * num_rows * 4);
    return mat;
}

PointCloudPtr loadRGBD2Cloud(string depth_file, string reg_file)
{
  cv::Mat depthMat = reload_32f_image(depth_file);
  cv::Mat imageMat = cv::imread(reg_file);
  PointCloudBuilder * builder = new PointCloudBuilder(depthMat, imageMat);
  PointCloudPtr cloud = builder->getPointCloud();
  delete builder;
  return cloud;
}

PointCloudPtr loadRGBD2Cloud(int n)
{
  char buffer[20];
  sprintf(buffer, "%d", n); 
  return loadRGBD2Cloud(prefixDepth+buffer+suffixDepth, prefixReg+buffer+suffixReg);
}
