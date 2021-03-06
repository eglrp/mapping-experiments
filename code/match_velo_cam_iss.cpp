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
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>

#include "load_velodyne.h"
#include "load_stereo_cam.h"

using namespace std;

int main(int argc, char** argv) {

    cout << "loading velodyne frame 0 from file" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame0_in_cam0_frame_ptr = loadVelFrameInCam0Ref(0);

    cout << "loading stereo frame 1 from file" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_frame0_in_cam0_frame_ptr = loadCamFrameInCamiRef(1);

    std::cout << "Saved " << velo_frame0_in_cam0_frame_ptr->points.size () << " data points from frame 0"
              << std::endl;
    std::cout << "Saved " << cam_frame0_in_cam0_frame_ptr->points.size () << " data points from frame 1"
              << std::endl;

    double model_resolution = 1.0;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_frame0_ptr (new pcl::PointCloud<pcl::PointXYZ>);

// Compute model_resolution
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (6 * model_resolution);
    iss_detector.setNonMaxRadius (4 * model_resolution);
    iss_detector.setThreshold21 (0.975);
    iss_detector.setThreshold32 (0.975);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (4);
    iss_detector.setInputCloud (velo_frame0_in_cam0_frame_ptr);
    iss_detector.compute (*keypoints_frame0_ptr);

    pcl::console::print_highlight ("Detected %zd points in frame 0\n", keypoints_frame0_ptr->size ());
//    pcl::PointIndicesConstPtr keypoints_indices_frame0 = detector.getKeypointsIndices ();
//    if (!keypoints_indices_frame0->indices.empty ())
//    {
////        pcl::io::savePCDFile ("keypoints.pcd", *cloud, keypoints_indices->indices, true);
//        pcl::console::print_info ("Saved keypoints to keypoints.pcd\n");
//    }
//    else
//        pcl::console::print_warn ("Keypoints indices are empty!\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_frame1_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    iss_detector.setInputCloud (cam_frame0_in_cam0_frame_ptr);
//    detector.setRadius(2.0);
    iss_detector.compute (*keypoints_frame1_ptr);
    pcl::console::print_highlight ("Detected %zd points in frame 1\n", keypoints_frame1_ptr->size ());



    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> frame0_cam0_color_handler(velo_frame0_in_cam0_frame_ptr, 0,
                                                                                                    0, 255);
    viewer.addPointCloud(velo_frame0_in_cam0_frame_ptr, frame0_cam0_color_handler, "frame 0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame 0");
////    viewer.addCoordinateSystem (1.0f, "global");
////    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
////    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters();
//    setViewerPose(viewer, range_image.getTransformationToWorldSystem());

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_frame0_cam0_color_handler(keypoints_frame0_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_frame0_ptr, keypoints_frame0_cam0_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
//    pcl::visualization::PCLVisualizer viewer("3D Viewer");
//    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> frame1_cam0_color_handler(cam_frame0_in_cam0_frame_ptr, 255,
                                                                                                                0, 0);
    viewer.addPointCloud(cam_frame0_in_cam0_frame_ptr, frame1_cam0_color_handler, "frame 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame 1");
////    viewer.addCoordinateSystem (1.0f, "global");
////    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
////    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
//    viewer.initCameraParameters();
//    setViewerPose(viewer, range_image.getTransformationToWorldSystem());

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_frame1_cam0_color_handler(keypoints_frame1_ptr, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_frame1_ptr, keypoints_frame1_cam0_color_handler, "keypoints 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints 1");


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (keypoints_frame0_ptr);

    for (int i = 0; i < 50; ++i) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_source);
        icp.setInputTarget(keypoints_frame1_ptr);
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

