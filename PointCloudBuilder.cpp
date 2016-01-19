//
// Created by 郭嘉丞 on 15/9/12.
//

#include "PointCloudBuilder.h"
#include "KinectParameters.h"
#include <cstdio>

using std::cerr;
using std::cout;
using std::endl;

PointCloudBuilder::PointCloudBuilder(const cv::Mat &depthMatrix, const cv::Mat &imageMatrix)
        : depthMat(depthMatrix), imageMat(imageMatrix), originalImage(imageMatrix.clone())
{
}

PointCloudPtr PointCloudBuilder::getPointCloud()
{
	if(!pointCloud)
	{
		pointCloud = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
		buildPointCloud();
	}
	return pointCloud;
}

void PointCloudBuilder::buildPointCloud()
{
    depthMat = depthMat * depthToZ[0] + depthToZ[1];
    for (int y = 0; y < imageMat.rows; y++)
    {
        for (int x = 0; x < imageMat.cols; x++)
        {
            double Z = depthMat.at<float>(y, x);
            if (Z < MIN_DEPTH_CM)
            {
                continue;
            }

            cv::Mat solvedXY = getPointXY(x, y, Z, projectionParameter);
            if(solvedXY.rows == 0 || solvedXY.cols == 0) continue;
            double X = solvedXY.at<double>(0);
            double Y = solvedXY.at<double>(1);
            pcl::PointXYZRGB newPoint;
            newPoint.b = imageMat.at<cv::Vec3b>(y, x)[0];
            newPoint.g = imageMat.at<cv::Vec3b>(y, x)[1];
            newPoint.r = imageMat.at<cv::Vec3b>(y, x)[2];
            newPoint.x = (float) X;
            newPoint.y = (float) Y;
            newPoint.z = (float) Z;
            pointCloud->points.push_back(newPoint);
        }
    }
    pointCloud->width = (int) pointCloud->points.size();
    pointCloud->height = 1;
    cout << "Build success" << endl;
}

cv::Mat PointCloudBuilder::getPointXY(int pixelx, int pixely, double depth, double p[3][4])
{
    double matrixAval[2][2] = {
            {p[0][0] - p[2][0] * pixelx, p[0][1] - p[2][1] * pixelx},
            {p[1][0] - p[2][0] * pixely, p[1][1] - p[2][1] * pixely}
    };
    cv::Mat matrixA(2, 2, CV_64FC1, &matrixAval);
    double matrixbval[2] = {
            p[2][2] * depth * pixelx + p[2][3] * pixelx - p[0][2] * depth - p[0][3],
            p[2][2] * depth * pixely + p[2][3] * pixely - p[1][2] * depth - p[1][3]
    };
    cv::Mat matrixb(2, 1, CV_64FC1, &matrixbval);
    cv::Mat solvedXY;
    if (!cv::solve(matrixA, matrixb, solvedXY))
    {
        cerr << "singular matrix A at x,y" << pixelx << " " << pixely << endl;
        return cv::Mat();
    }
    return solvedXY;
}
