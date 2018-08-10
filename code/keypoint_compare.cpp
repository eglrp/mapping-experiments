//
// Created by boroson on 7/30/18.
//

#include <iostream>
#include <fstream>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/common/geometry.h>
#include <pcl/keypoints/iss_3d.h>

#include "keypoint_compare.h"

std::vector<int>  matchNarfDescriptors(pcl::PointCloud<pcl::Narf36> velo_features, pcl::PointCloud<pcl::Narf36> cam_features);


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

    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_cloud2_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    detector.setInputCloud (cloud2_ptr);
//    detector.setRadius(2.0);
    detector.compute (*keypoints_cloud2_ptr);
    pcl::console::print_highlight ("Detected %zd points in cloud 2\n", keypoints_cloud2_ptr->size ());

    std::ofstream outfile1;
    std::string filename1 = filename + "_1.csv";
    outfile1.open(filename1);
    outfile1 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared" << std::endl;

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
            outfile1 << i << ", " << p1[0] << ", " << p1[1] << ", " << p1[2] <<", " << pointIdxNKNSearch[0] << ", " << p2[0] << ", " << p2[1] << ", " << p2[2] << ", " << pointNKNSquaredDistance[0] << std::endl;
        }
    }
    outfile1.close();

    std::ofstream outfile2;
    std::string filename2 = filename + "_2.csv";
    outfile2.open(filename2);
    outfile2 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared" << std::endl;

    // Create KD tree to do nearest neighbor search
//    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(keypoints_cloud1_ptr);

    // For each point in cloud 2, find corresponding point(s) in cloud 1
    for (size_t i = 0; i < keypoints_cloud2_ptr->points.size(); ++i)
    {
        Eigen::Vector3f p2  = keypoints_cloud2_ptr->points[i].getVector3fMap();

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if ( kdtree.nearestKSearch (keypoints_cloud2_ptr->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            Eigen::Vector3f p1  = keypoints_cloud1_ptr->points[pointIdxNKNSearch[0]].getVector3fMap();
            outfile2 << pointIdxNKNSearch[0] << ", " << p1[0] << ", " << p1[1] << ", " << p1[2] <<", " << i << ", " << p2[0] << ", " << p2[1] << ", " << p2[2] << ", " << pointNKNSquaredDistance[0] << std::endl;
        }
    }
    outfile2.close();
}

void compare_iss_keypts(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, std::string filename)
{
    double model_resolution = 1.0;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud1_ptr (new pcl::PointCloud<pcl::PointXYZ>);

// Compute model_resolution
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (6 * model_resolution);
    iss_detector.setNonMaxRadius (4 * model_resolution);
    iss_detector.setThreshold21 (0.975);
    iss_detector.setThreshold32 (0.975);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (4);
    iss_detector.setInputCloud (cloud1_ptr);
    iss_detector.compute (*keypoints_cloud1_ptr);

    pcl::console::print_highlight ("Detected %zd points in cloud 1\n", keypoints_cloud1_ptr->size ());
//    pcl::PointIndicesConstPtr keypoints_indices_frame0 = detector.getKeypointsIndices ();
//    if (!keypoints_indices_frame0->indices.empty ())
//    {
////        pcl::io::savePCDFile ("keypoints.pcd", *cloud, keypoints_indices->indices, true);
//        pcl::console::print_info ("Saved keypoints to keypoints.pcd\n");
//    }
//    else
//        pcl::console::print_warn ("Keypoints indices are empty!\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud2_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    iss_detector.setInputCloud (cloud2_ptr);
//    detector.setRadius(2.0);
    iss_detector.compute (*keypoints_cloud2_ptr);
    pcl::console::print_highlight ("Detected %zd points in cloud 2\n", keypoints_cloud2_ptr->size ());

    std::ofstream outfile1;
    std::string filename1 = filename + "_1.csv";
    outfile1.open(filename1);
    outfile1 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared" << std::endl;

    // Create KD tree to do nearest neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(keypoints_cloud2_ptr);

    // For each point in cloud 1, find corresponding point(s) in cloud 2
    for (size_t i = 0; i < keypoints_cloud1_ptr->points.size(); ++i)
    {
        Eigen::Vector3f p1  = keypoints_cloud1_ptr->points[i].getVector3fMap();

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if ( kdtree.nearestKSearch (keypoints_cloud1_ptr->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            Eigen::Vector3f p2  = keypoints_cloud2_ptr->points[pointIdxNKNSearch[0]].getVector3fMap();
            outfile1 << i << ", " << p1[0] << ", " << p1[1] << ", " << p1[2] <<", " << pointIdxNKNSearch[0] << ", " << p2[0] << ", " << p2[1] << ", " << p2[2] << ", " << pointNKNSquaredDistance[0] << std::endl;
        }
    }
    outfile1.close();

    std::ofstream outfile2;
    std::string filename2 = filename + "_2.csv";
    outfile2.open(filename2);
    outfile2 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared" << std::endl;

    // Create KD tree to do nearest neighbor search
//    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(keypoints_cloud1_ptr);

    // For each point in cloud 2, find corresponding point(s) in cloud 1
    for (size_t i = 0; i < keypoints_cloud2_ptr->points.size(); ++i)
    {
        Eigen::Vector3f p2  = keypoints_cloud2_ptr->points[i].getVector3fMap();

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if ( kdtree.nearestKSearch (keypoints_cloud2_ptr->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            Eigen::Vector3f p1  = keypoints_cloud1_ptr->points[pointIdxNKNSearch[0]].getVector3fMap();
            outfile2 << pointIdxNKNSearch[0] << ", " << p1[0] << ", " << p1[1] << ", " << p1[2] <<", " << i << ", " << p2[0] << ", " << p2[1] << ", " << p2[2] << ", " << pointNKNSquaredDistance[0] << std::endl;
        }
    }
    outfile2.close();
}



void compare_narf_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_ptr, std::string filename_keypt_dist, std::string filename_desc) {
    // -----------------------------------------------
    // -----Create RangeImage from the Frame 0 PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    float angular_resolution = pcl::deg2rad(0.5f);
    float support_size = 0.25f; //0.2f;
    bool rotation_invariant = true;
    Eigen::Affine3f cam_sensor_pose(Eigen::Affine3f::Identity());
    boost::shared_ptr<pcl::RangeImage> range_image_cloud1_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image_cloud1 = *range_image_cloud1_ptr;
    range_image_cloud1.createFromPointCloud(*cloud1_ptr, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                                 cam_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range,
                                                 border_size);
//    range_image.integrateFarRanges (far_ranges);
//    if (setUnseenToMaxRange)
//        range_image.setUnseenToMaxRange();

    //    // --------------------------
//    // -----Show range image-----
//    // --------------------------
//    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//    range_image_widget.showRangeImage (range_image_cloud1);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_cloud1_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_cloud1_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image_cloud1);
    narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices_cloud1;
    narf_keypoint_detector.compute(keypoint_indices_cloud1);
    std::cout << "Found " << keypoint_indices_cloud1.points.size() << " key points.\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud1 = *keypoints_cloud1_ptr;
    keypoints_cloud1.points.resize(keypoint_indices_cloud1.points.size());
    for (size_t i = 0; i < keypoint_indices_cloud1.points.size(); ++i)
        keypoints_cloud1.points[i].getVector3fMap() = range_image_cloud1.points[keypoint_indices_cloud1.points[i]].getVector3fMap();

    // ------------------------------------------------------
    // -----Extract NARF descriptors for interest points-----
    // ------------------------------------------------------
    std::vector<int> keypoint_indices_vector_cloud1;
    keypoint_indices_vector_cloud1.resize(keypoint_indices_cloud1.points.size());
    for (unsigned int i = 0; i < keypoint_indices_vector_cloud1.size(); ++i) // This step is necessary to get the right vector type
        keypoint_indices_vector_cloud1[i] = keypoint_indices_cloud1.points[i];
    pcl::NarfDescriptor narf_descriptor(&range_image_cloud1, &keypoint_indices_vector_cloud1);
    narf_descriptor.getParameters().support_size = support_size;
    narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
    pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors_cloud1_ptr(new pcl::PointCloud<pcl::Narf36>);
    pcl::PointCloud<pcl::Narf36> &narf_descriptors_cloud1 = *narf_descriptors_cloud1_ptr;
    narf_descriptor.compute(narf_descriptors_cloud1);
    cout << "Extracted " << narf_descriptors_cloud1.size() << " descriptors for "
         << keypoint_indices_cloud1.points.size() << " keypoints.\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr narf_descriptor_keypoints_cloud1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &narf_descriptor_keypoints_cloud1 = *narf_descriptor_keypoints_cloud1_ptr;
    narf_descriptor_keypoints_cloud1.points.resize(narf_descriptors_cloud1.points.size());
    for (size_t i = 0; i < narf_descriptors_cloud1.points.size(); ++i) {
        narf_descriptor_keypoints_cloud1.points[i].x = narf_descriptors_cloud1.points[i].x;
        narf_descriptor_keypoints_cloud1.points[i].y = narf_descriptors_cloud1.points[i].y;
        narf_descriptor_keypoints_cloud1.points[i].z = narf_descriptors_cloud1.points[i].z;
    }

    // -----------------------------------------------
    // -----Create RangeImage from PointCloud 2-----
    // -----------------------------------------------
    boost::shared_ptr<pcl::RangeImage> range_image_cloud2_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image_cloud2 = *range_image_cloud2_ptr;
    range_image_cloud2.createFromPointCloud(*cloud2_ptr, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                            cam_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range,
                                            border_size);
//    range_image.integrateFarRanges (far_ranges);
//    if (setUnseenToMaxRange)
//        range_image.setUnseenToMaxRange();

    //    // --------------------------
//    // -----Show range image-----
//    // --------------------------
//    pcl::visualization::RangeImageVisualizer range_image_widget2 ("Range image 2");
//    range_image_widget2.showRangeImage (range_image_cloud2);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_cloud2_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector2;
    narf_keypoint_detector2.setRangeImageBorderExtractor(&range_image_cloud2_border_extractor);
    narf_keypoint_detector2.setRangeImage(&range_image_cloud2);
    narf_keypoint_detector2.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices_cloud2;
    narf_keypoint_detector2.compute(keypoint_indices_cloud2);
    std::cout << "Found " << keypoint_indices_cloud2.points.size() << " key points.\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &keypoints_cloud2 = *keypoints_cloud2_ptr;
    keypoints_cloud2.points.resize(keypoint_indices_cloud2.points.size());
    for (size_t i = 0; i < keypoint_indices_cloud2.points.size(); ++i)
        keypoints_cloud2.points[i].getVector3fMap() = range_image_cloud2.points[keypoint_indices_cloud2.points[i]].getVector3fMap();

    // ------------------------------------------------------
    // -----Extract NARF descriptors for interest points-----
    // ------------------------------------------------------
    std::vector<int> keypoint_indices_vector_cloud2;
    keypoint_indices_vector_cloud2.resize(keypoint_indices_cloud2.points.size());
    for (unsigned int i = 0; i < keypoint_indices_vector_cloud2.size(); ++i) // This step is necessary to get the right vector type
        keypoint_indices_vector_cloud2[i] = keypoint_indices_cloud2.points[i];
    pcl::NarfDescriptor narf_descriptor2(&range_image_cloud2, &keypoint_indices_vector_cloud2);
    narf_descriptor2.getParameters().support_size = support_size;
    narf_descriptor2.getParameters().rotation_invariant = rotation_invariant;
    pcl::PointCloud<pcl::Narf36>::Ptr narf_descriptors_cloud2_ptr(new pcl::PointCloud<pcl::Narf36>);
    pcl::PointCloud<pcl::Narf36> &narf_descriptors_cloud2 = *narf_descriptors_cloud2_ptr;
    narf_descriptor2.compute(narf_descriptors_cloud2);
    cout << "Extracted " << narf_descriptors_cloud2.size() << " descriptors for "
         << keypoint_indices_cloud2.points.size() << " keypoints.\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr narf_descriptor_keypoints_cloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &narf_descriptor_keypoints_cloud2 = *narf_descriptor_keypoints_cloud2_ptr;
    narf_descriptor_keypoints_cloud2.points.resize(narf_descriptors_cloud2.points.size());
    for (size_t i = 0; i < narf_descriptors_cloud2.points.size(); ++i) {
        narf_descriptor_keypoints_cloud2.points[i].x = narf_descriptors_cloud2.points[i].x;
        narf_descriptor_keypoints_cloud2.points[i].y = narf_descriptors_cloud2.points[i].y;
        narf_descriptor_keypoints_cloud2.points[i].z = narf_descriptors_cloud2.points[i].z;
    }

    std::ofstream outfile1;
    std::string filename1 = filename_keypt_dist + "_1.csv";
    outfile1.open(filename1);
    outfile1 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared, desc_dist" << std::endl;

    // Create KD tree to do nearest neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(narf_descriptor_keypoints_cloud2_ptr);

    // For each point in cloud 1, find corresponding point(s) in cloud 2
    for (size_t i = 0; i < narf_descriptors_cloud1.points.size(); ++i) {
        pcl::PointXYZ p1(narf_descriptors_cloud1.points[i].x, narf_descriptors_cloud1.points[i].y, narf_descriptors_cloud1.points[i].z);

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(narf_descriptor_keypoints_cloud1.points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            pcl::PointXYZ p2(narf_descriptors_cloud2_ptr->points[pointIdxNKNSearch[0]].x, narf_descriptors_cloud2_ptr->points[pointIdxNKNSearch[0]].y, narf_descriptors_cloud2_ptr->points[pointIdxNKNSearch[0]].z);
            float dist = pcl::L2_Norm (narf_descriptors_cloud1.points[i].descriptor, narf_descriptors_cloud2_ptr->points[pointIdxNKNSearch[0]].descriptor, 36);
            outfile1 << i << ", " << p1.x << ", " << p1.y << ", " << p1.z << ", " << pointIdxNKNSearch[0] << ", "
                    <<  p2.x << ", " << p2.y << ", " << p2.z << ", " << pointNKNSquaredDistance[0] << ", " << dist << std::endl;
        }
    }
    outfile1.close();

    std::ofstream outfile2;
    std::string filename2 = filename_desc + "_1.csv";
    outfile2.open(filename2);
    outfile2 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared, desc_dist" << std::endl;

    // ----------------------------------------------------
    // -----Match NARF descriptors and display matches-----
    // ----------------------------------------------------
    std::vector<int> match_inds;
    match_inds = matchNarfDescriptors(narf_descriptors_cloud1, narf_descriptors_cloud2);
    for (size_t i = 0; i < keypoint_indices_cloud1.points.size(); ++i)
    {
        if (match_inds[i] < narf_descriptors_cloud2.points.size()) {
            pcl::PointXYZ p1(narf_descriptors_cloud1.points[i].x, narf_descriptors_cloud1.points[i].y, narf_descriptors_cloud1.points[i].z);
            pcl::PointXYZ p2(narf_descriptors_cloud2.points[match_inds[i]].x, narf_descriptors_cloud2.points[match_inds[i]].y, narf_descriptors_cloud2.points[match_inds[i]].z);
            float dist = pcl::L2_Norm (narf_descriptors_cloud1.points[i].descriptor, narf_descriptors_cloud2.points[match_inds[i]].descriptor, 36);
            outfile2 << i << ", " << p1.x << ", " << p1.y << ", " << p1.z << ", " << match_inds[i] << ", "
                    << p2.x << ", " << p2.y << ", " << p2.z << ", " << pcl::geometry::distance(p1, p2) << ", " << dist << std::endl;

        }
    }
    outfile2.close();

    std::ofstream outfile3;
    std::string filename3 = filename_keypt_dist + "_2.csv";
    outfile3.open(filename3);
    outfile3 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared, desc_dist" << std::endl;

    // Create KD tree to do nearest neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;
    kdtree2.setInputCloud(narf_descriptor_keypoints_cloud1_ptr);

    // For each point in cloud 1, find corresponding point(s) in cloud 2
    for (size_t i = 0; i < narf_descriptors_cloud2.points.size(); ++i) {
        pcl::PointXYZ p2(narf_descriptors_cloud2.points[i].x, narf_descriptors_cloud2.points[i].y, narf_descriptors_cloud2.points[i].z);

        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree2.nearestKSearch(narf_descriptor_keypoints_cloud2.points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            pcl::PointXYZ p1(narf_descriptors_cloud1_ptr->points[pointIdxNKNSearch[0]].x, narf_descriptors_cloud1_ptr->points[pointIdxNKNSearch[0]].y, narf_descriptors_cloud1_ptr->points[pointIdxNKNSearch[0]].z);
            float dist = pcl::L2_Norm (narf_descriptors_cloud2.points[i].descriptor, narf_descriptors_cloud1_ptr->points[pointIdxNKNSearch[0]].descriptor, 36);
            outfile3 << pointIdxNKNSearch[0] << ", " << p1.x << ", " << p1.y << ", " << p1.z << ", " << i << ", "
                     <<  p2.x << ", " << p2.y << ", " << p2.z << ", " << pointNKNSquaredDistance[0] << ", " << dist << std::endl;
        }
    }
    outfile3.close();

    std::ofstream outfile4;
    std::string filename4 = filename_desc + "_2.csv";
    outfile4.open(filename4);
    outfile4 << "i1, x1, y1, z1, i2, x2, y2, z2, dist_squared, desc_dist" << std::endl;

    // ----------------------------------------------------
    // -----Match NARF descriptors and display matches-----
    // ----------------------------------------------------
    std::vector<int> match_inds2;
    match_inds = matchNarfDescriptors(narf_descriptors_cloud2, narf_descriptors_cloud1);
    for (size_t i = 0; i < keypoint_indices_cloud2.points.size(); ++i)
    {
        if (match_inds[i] < narf_descriptors_cloud1.points.size()) {
            pcl::PointXYZ p1(narf_descriptors_cloud1.points[match_inds[i]].x, narf_descriptors_cloud1.points[match_inds[i]].y, narf_descriptors_cloud1.points[match_inds[i]].z);
            pcl::PointXYZ p2(narf_descriptors_cloud2.points[i].x, narf_descriptors_cloud2.points[i].y, narf_descriptors_cloud2.points[i].z);
            float dist = pcl::L2_Norm (narf_descriptors_cloud1.points[match_inds[i]].descriptor, narf_descriptors_cloud2.points[i].descriptor, 36);
            outfile4 << match_inds[i] << ", " << p1.x << ", " << p1.y << ", " << p1.z << ", " << i << ", "
                     << p2.x << ", " << p2.y << ", " << p2.z << ", " << pcl::geometry::distance(p1, p2) << ", " << dist << std::endl;

        }
    }
    outfile4.close();
}

// Compute feature matches based on descriptors only.
// Returns vector of indices into cam feature list to match each lidar feature.
std::vector<int>  matchNarfDescriptors(pcl::PointCloud<pcl::Narf36> velo_features, pcl::PointCloud<pcl::Narf36> cam_features) //, std::vector<int> velo_keypts, std::vector<int> cam_keypts, pcl::RangeImage velo_range_img, pcl::RangeImage cam_range_img)
{
    std::vector<int> ind;
    for (size_t i=0; i<velo_features.points.size (); ++i)
    {
        float best_dist = 1e6;//0.02*36;
        int best_ind = cam_features.points.size();
        for (size_t j=0; j<cam_features.points.size (); ++j)
        {
            float dist = pcl::L2_Norm (velo_features.points[i].descriptor, cam_features.points[j].descriptor, 36);
//            float dist = pow(velo_features.points[i].x - cam_features.points[j].x, 2) + pow(velo_features.points[i].y - cam_features.points[j].y, 2) + pow(velo_features.points[i].z - cam_features.points[j].z, 2);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_ind = j;
            }
        }

        ind.push_back(best_ind);
        if (best_ind < cam_features.points.size()) {
            float desc = pcl::L1_Norm(velo_features.points[i].descriptor, cam_features.points[best_ind].descriptor, 36);
//            cout << desc << endl;
        }
    }

    return ind;
}
