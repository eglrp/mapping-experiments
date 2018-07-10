//
// Created by boroson on 4/18/18.
//

/*
 * Function to read velodyne data from a binary data file and create
 * octomap with it
 */

#include <vector>
#include <string>

//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>

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

#include "load_velodyne.h"

using namespace std;

int main(int argc, char** argv) {

    cout << "loading velodyne frame 0 from file" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame0_in_cam0_frame_ptr = loadVelFrameInCam0Ref(0);

    cout << "loading velodyne frame 1 from file" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame1_in_cam0_frame_ptr = loadVelFrameInCam0Ref(1);

//    // Estimate normals on velo dataset (from http://pointclouds.org/documentation/tutorials/normal_estimation.php)
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_velo;
//    ne_velo.setInputCloud (cloud_velo);
//
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//    ne_velo.setSearchMethod (tree);
//
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_velo_normals (new pcl::PointCloud<pcl::Normal>);
//    ne_velo.setRadiusSearch (0.10); //meters
//
//    // Compute the features
//    ne_velo.compute (*cloud_velo_normals);

//    // Estimate normals on cam dataset (from http://pointclouds.org/documentation/tutorials/normal_estimation.php)
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_cam;
//    ne_cam.setInputCloud (cloud_cam);
//
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cam (new pcl::search::KdTree<pcl::PointXYZ> ());
//    ne_cam.setSearchMethod (tree_cam);
//
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_cam_normals (new pcl::PointCloud<pcl::Normal>);
//    ne_cam.setRadiusSearch (0.10); //meters
//
//    // Compute the features
//    ne_cam.compute (*cloud_cam_normals);

    std::cout << "Saved " << velo_frame0_in_cam0_frame_ptr->points.size () << " data points from frame 0"
              << std::endl;
    std::cout << "Saved " << velo_frame1_in_cam0_frame_ptr->points.size () << " data points from frame 1"
              << std::endl;

//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_velo_pt_norm (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cam_pt_norm (new pcl::PointCloud<pcl::PointNormal>);
//
//    for(int i = 0; i < cloud_velo_normals->points.size(); i++)
//    {
//        pcl::PointNormal pt;
//        pt.x = cloud_velo->points[i].x;
//        pt.y = cloud_velo->points[i].y;
//        pt.z = cloud_velo->points[i].z;
//
//        pt.curvature = cloud_velo_normals->points[i].curvature;
//        pt.normal_x = cloud_velo_normals->points[i].normal_x;
//        pt.normal_y = cloud_velo_normals->points[i].normal_y;
//        pt.normal_z = cloud_velo_normals->points[i].normal_z;
//        cloud_velo_pt_norm->push_back(pt);
//    }

//    for(int i = 0; i < cloud_cam_normals->points.size(); i++)
//    {
//        pcl::PointNormal pt;
//        pt.x = cloud_cam->points[i].x;
//        pt.y = cloud_cam->points[i].y;
//        pt.z = cloud_cam->points[i].z;
//
//        pt.curvature = cloud_cam_normals->points[i].curvature;
//        pt.normal_x = cloud_cam_normals->points[i].normal_x;
//        pt.normal_y = cloud_cam_normals->points[i].normal_y;
//        pt.normal_z = cloud_cam_normals->points[i].normal_z;
//        cloud_cam_pt_norm->push_back(pt);
//    }

//    //remove NAN points from the cloud
//    std::vector<int> indices;
//    pcl::removeNaNNormalsFromPointCloud(*cloud_velo_pt_norm,*cloud_velo_pt_norm, indices);
//    pcl::removeNaNNormalsFromPointCloud(*cloud_cam_pt_norm,*cloud_cam_pt_norm, indices);

    // -----------------------------------------------
    // -----Create RangeImage from the Frame 0 PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    float angular_resolution = pcl::deg2rad(0.5f);
    float support_size = 0.25f; //0.2f;
    Eigen::Affine3f cam_sensor_pose(Eigen::Affine3f::Identity());
    boost::shared_ptr<pcl::RangeImage> range_image_frame0_cam0_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image_frame0_cam0 = *range_image_frame0_cam0_ptr;
    range_image_frame0_cam0.createFromPointCloud(*velo_frame0_in_cam0_frame_ptr, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     cam_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range,
                                     border_size);
//    range_image.integrateFarRanges (far_ranges);
//    if (setUnseenToMaxRange)
//        range_image.setUnseenToMaxRange();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_frame0_cam0_color_handler(range_image_frame0_cam0_ptr, 0,
                                                                                                    0, 255);
    viewer.addPointCloud(range_image_frame0_cam0_ptr, range_image_frame0_cam0_color_handler, "range image 0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image 0");
////    viewer.addCoordinateSystem (1.0f, "global");
////    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
////    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters();
//    setViewerPose(viewer, range_image.getTransformationToWorldSystem());
//
//    // --------------------------
//    // -----Show range image-----
//    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image_frame0_cam0);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_frame0_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_frame0_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image_frame0_cam0);
    narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices_frame0_cam0;
    narf_keypoint_detector.compute(keypoint_indices_frame0_cam0);
    std::cout << "Found " << keypoint_indices_frame0_cam0.points.size() << " key points.\n";

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_frame0_cam0_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &keypoints_frame0_cam0 = *keypoints_frame0_cam0_ptr;
    keypoints_frame0_cam0.points.resize(keypoint_indices_frame0_cam0.points.size());
    for (size_t i = 0; i < keypoint_indices_frame0_cam0.points.size(); ++i)
        keypoints_frame0_cam0.points[i].getVector3fMap() = range_image_frame0_cam0.points[keypoint_indices_frame0_cam0.points[i]].getVector3fMap();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_frame0_cam0_color_handler(keypoints_frame0_cam0_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_frame0_cam0_ptr, keypoints_frame0_cam0_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    // -----------------------------------------------
    // -----Create RangeImage from the Frame 1 PointCloud-----
    // -----------------------------------------------

    boost::shared_ptr<pcl::RangeImage> range_image_frame1_cam0_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image_frame1_cam0 = *range_image_frame1_cam0_ptr;
    range_image_frame1_cam0.createFromPointCloud(*velo_frame1_in_cam0_frame_ptr, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                                 cam_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range,
                                                 border_size);
//    range_image.integrateFarRanges (far_ranges);
//    if (setUnseenToMaxRange)
//        range_image.setUnseenToMaxRange();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
//    pcl::visualization::PCLVisualizer viewer("3D Viewer");
//    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_frame1_cam0_color_handler(range_image_frame1_cam0_ptr, 255,
                                                                                                                0, 0);
    viewer.addPointCloud(range_image_frame1_cam0_ptr, range_image_frame1_cam0_color_handler, "range image 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image 1");
////    viewer.addCoordinateSystem (1.0f, "global");
////    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
////    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters();
//    setViewerPose(viewer, range_image.getTransformationToWorldSystem());
//
//    // --------------------------
//    // -----Show range image-----
//    // --------------------------
//    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//    range_image_widget.showRangeImage (range_image_frame0_cam0);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_frame1_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector_frame1;
    narf_keypoint_detector_frame1.setRangeImageBorderExtractor(&range_image_frame1_border_extractor);
    narf_keypoint_detector_frame1.setRangeImage(&range_image_frame1_cam0);
    narf_keypoint_detector_frame1.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices_frame1_cam0;
    narf_keypoint_detector_frame1.compute(keypoint_indices_frame1_cam0);
    std::cout << "Found " << keypoint_indices_frame1_cam0.points.size() << " key points.\n";

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_frame1_cam0_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &keypoints_frame1_cam0 = *keypoints_frame1_cam0_ptr;
    keypoints_frame1_cam0.points.resize(keypoint_indices_frame1_cam0.points.size());
    for (size_t i = 0; i < keypoint_indices_frame1_cam0.points.size(); ++i)
        keypoints_frame1_cam0.points[i].getVector3fMap() = range_image_frame1_cam0.points[keypoint_indices_frame1_cam0.points[i]].getVector3fMap();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_frame1_cam0_color_handler(keypoints_frame1_cam0_ptr, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_frame1_cam0_ptr, keypoints_frame1_cam0_color_handler, "keypoints 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints 1");


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (keypoints_frame0_cam0_ptr);

    for (int i = 0; i < 50; ++i) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_source);
        icp.setInputTarget(keypoints_frame1_cam0_ptr);
        icp.setTransformationEpsilon(1e-13);
        icp.setEuclideanFitnessEpsilon(0.1);
        icp.setMaxCorrespondenceDistance (20.0);
//        icp.setRANSACOutlierRejectionThreshold (1.0);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        *cloud_source = Final;
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                  icp.getFitnessScore(1.0) << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_final_cam0_color_handler(cloud_source, 255, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_source, keypoints_final_cam0_color_handler, "keypoints final");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints final");

//    pcl::visualization::PCLVisualizer viewer("3D Viewer");
//    viewer.setBackgroundColor(1, 1, 1);
////    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> frame_final_color_handler(cloud_source, 255,
////                                                                                                    0, 0);
////    viewer.addPointCloud(cloud_source, frame_final_color_handler, "frame_final");
////    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame_final");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> frame0_color_handler(velo_frame0_in_cam0_frame_ptr, 0,
//                                                                                         0, 255);
//    viewer.addPointCloud(velo_frame0_in_cam0_frame_ptr, frame0_color_handler, "frame0");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame0");
//////    viewer.addCoordinateSystem (1.0f, "global");
//////    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
//////    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
//    viewer.initCameraParameters();
////    pcl::setViewerPose(viewer, velo_frame0_in_cam0_frame_ptr->getTransformationToWorldSystem());
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> frame1_color_handler(velo_frame1_in_cam0_frame_ptr, 0,
//                                                                                         255, 0);
//    viewer.addPointCloud(velo_frame1_in_cam0_frame_ptr, frame1_color_handler, "frame1");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame1");

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
//        range_image_widget.spinOnce ();  // process GUI events
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }

    return 0;
}

