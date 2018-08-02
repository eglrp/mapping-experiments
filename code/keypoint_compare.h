//
// Created by boroson on 7/30/18.
//


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifndef CODE_KEYPOINT_COMPARE_H
#define CODE_KEYPOINT_COMPARE_H

void compare_harris_keypts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string filename);

#endif //CODE_KEYPOINT_COMPARE_H
