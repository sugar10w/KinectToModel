
#ifndef _OBJECTSCAN_LOADRGBD_
#define _OBJECTSCAN_LOADRGBD_

#include<string>
#include"common.h"

const std::string prefixDepth = "depth", suffixDepth = ".bin";
const std::string prefixReg = "registered", suffixReg = ".png";

PointCloudPtr loadRGBD2Cloud(int n);
PointCloudPtr loadRGBD2Cloud(std::string depth_file, std::string reg_file);

#endif //_OBJECTSCAN_LOADRGBD_
