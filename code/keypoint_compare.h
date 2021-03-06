//
// Created by boroson on 7/30/18.
//


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifndef CODE_KEYPOINT_COMPARE_H
#define CODE_KEYPOINT_COMPARE_H

void compare_harris_keypts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr intersection);

void compare_harris_keypts_frametoframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, Eigen::Matrix4f t1, Eigen::Matrix4f t2, std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr intersection);

void compare_iss_keypts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr intersection);

void compare_kpq_keypts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr intersection);

void compare_kpqas_keypts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr intersection);

void compare_narf_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, std::string filename_keypt_dist, std::string filename_desc, pcl::PointCloud<pcl::PointXYZ>::Ptr intersection);


#endif //CODE_KEYPOINT_COMPARE_H
