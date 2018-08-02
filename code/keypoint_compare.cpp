//
// Created by boroson on 7/30/18.
//

#include <iostream>
#include <fstream>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "keypoint_compare.h"

void compare_harris_keypts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, std::string filename)
{
    pcl::HarrisKeypoint3D <pcl::PointXYZ, pcl::PointXYZI> detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_cloud1_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    detector.setNonMaxSupression (true);
    detector.setInputCloud (cloud1_ptr);
    detector.setThreshold (1e-6);
    detector.setRadius(1.0);
//    pcl::StopWatch watch;
    detector.compute (*keypoints_cloud1_ptr);
    pcl::console::print_highlight ("Detected %zd points in cloud 1\n", keypoints_cloud1_ptr->size ());
//    pcl::PointIndicesConstPtr keypoints_indices_frame0 = detector.getKeypointsIndices ();
//    if (!keypoints_indices_frame0->indices.empty ())
//    {
////        pcl::io::savePCDFile ("keypoints.pcd", *cloud, keypoints_indices->indices, true);
//        pcl::console::print_info ("Saved keypoints to keypoints.pcd\n");
//    }
//    else
//        pcl::console::print_warn ("Keypoints indices are empty!\n");

    std::ofstream outfile;
    outfile.open(filename);
    outfile << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_cloud2_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    detector.setInputCloud (cloud2_ptr);
//    detector.setRadius(2.0);
    detector.compute (*keypoints_cloud2_ptr);
    pcl::console::print_highlight ("Detected %zd points in cloud 2\n", keypoints_cloud2_ptr->size ());

    // Create KD tree to do nearest neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(keypoints_cloud2_ptr);

    // For each point in cloud 1, find corresponding point(s) in cloud 2
    for (size_t i = 0; i < keypoints_cloud1_ptr->points.size(); ++i)
    {
        Eigen::Vector3f p1  = keypoints_cloud1_ptr->points[i].getVector3fMap();

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if ( kdtree.nearestKSearch (keypoints_cloud1_ptr->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            Eigen::Vector3f p2  = keypoints_cloud2_ptr->points[pointIdxNKNSearch[0]].getVector3fMap();
            outfile << i << ", " << p1[0] << ", " << p1[1] << ", " << p1[2] <<", " << pointIdxNKNSearch[0] << ", " << p2[0] << ", " << p2[1] << ", " << p2[2] << ", " << pointNKNSquaredDistance[0] << std::endl;
        }
    }
    outfile.close();
}