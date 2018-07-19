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
#include <pcl/registration/gicp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/harris_3d.h>

#include "load_velodyne.h"
#include "load_stereo_cam.h"

using namespace std;

int main(int argc, char** argv) {

    cout << "loading velodyne frame 0 from file" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame0_in_cam0_frame_ptr = loadVelFrameInCam0Ref(0);

    cout << "loading stereo frame 1 from file" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_frame0_in_cam0_frame_ptr = loadCamFrameInCamiRef(20);

    std::cout << "Saved " << velo_frame0_in_cam0_frame_ptr->points.size () << " data points from frame 0"
              << std::endl;
    std::cout << "Saved " << cam_frame0_in_cam0_frame_ptr->points.size () << " data points from frame 1"
              << std::endl;



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


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (velo_frame0_in_cam0_frame_ptr);

    for (int i = 0; i < 50; ++i) {
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_source);
        icp.setInputTarget(cam_frame0_in_cam0_frame_ptr);
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

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_cam0_color_handler(cloud_source, 255, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_source, final_cam0_color_handler, "keypoints final");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "keypoints final");


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

