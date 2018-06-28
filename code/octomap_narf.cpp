//
// Created by boroson on 4/18/18.
//

/*
 * Function to read velodyne data from a binary data file and create
 * octomap with it
 */

#include <vector>
#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap/Pointcloud.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/common/norms.h>

#include "matrix.h"

using namespace std;
using namespace octomap;

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
//float angular_resolution = 0.5f;
float support_size = 0.25f; //0.2f;
//pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

void read_scan(Pointcloud *pc, string filename);

vector<Matrix> loadPoses(string file_name);

pose6d transformToPose(Matrix t);

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

std::vector<int>  matchNarfDescriptors(pcl::PointCloud<pcl::Narf36> velo_features, pcl::PointCloud<pcl::Narf36> cam_features);

//void read_calib(cv::Mat& cal, string filename);

int main(int argc, char** argv) {

    cout << "loading velodyne octomap from file" << endl;

    OcTree velo_tree("velo_tree_frame0.bt");  // load from file

    cout << "loading stereo image octomap from file" << endl;

    OcTree cam_tree("velo_tree_frame20.bt");  // load from file

    string data_path = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/";
    string filename2 = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/000001.bin";
    string calib_file = "/home/boroson/data/kitti/dataset/sequences/00/calib.txt";
    string poses_file = "/home/boroson/data/kitti/dataset/poses/00.txt";

    pcl::PointCloud<PointType> cloud_velo;// (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointType> cloud_cam_orig; // (new pcl::PointCloud<pcl::PointXYZ>);

    // z -> x
    // x -> -y
    // y -> -z

    for (OcTree::leaf_iterator it = velo_tree.begin_leafs(),
                 end = velo_tree.end_leafs(); it != end; ++it) {
        if (it->getValue() > 0.0) {
//            cloud_velo.push_back(
//                    pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
            cloud_velo.push_back(
                    pcl::PointXYZ(-it.getCoordinate().y(), -it.getCoordinate().z(), it.getCoordinate().x()));
        }
    }

    int num_leafs = 0;

    for (OcTree::leaf_iterator it = cam_tree.begin_leafs(),
                 end = cam_tree.end_leafs(); it != end; ++it) {
        if (it->getValue() > 0.0) {
//            cloud_cam_orig.push_back(
//                    pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
            cloud_cam_orig.push_back(
                    pcl::PointXYZ(-it.getCoordinate().y(), -it.getCoordinate().z(), it.getCoordinate().x()));
            ++num_leafs;
        }
    }

    std::cout << "Got " << num_leafs << " points from cam tree.\n";

//    Eigen::Affine3f velo_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (cloud_velo.sensor_origin_[0],
//                                                                              cloud_velo.sensor_origin_[1],
//                                                                              cloud_velo.sensor_origin_[2])) *
//                        Eigen::Affine3f (cloud_velo.sensor_orientation_);

    Eigen::Affine3f velo_sensor_pose(Eigen::Affine3f::Identity());
    Eigen::Affine3f cam_sensor_pose(Eigen::Affine3f::Identity());

//    Eigen::Affine3f cam_transform(Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f(0, 0, 1)) *
//                                  Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f(1, 0, 0)));
    Eigen::Affine3f cam_transform(Eigen::Affine3f::Identity());
    pcl::PointCloud<pcl::PointXYZ> cloud_cam;
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(cloud_cam_orig, cloud_cam, cam_transform);

    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    float angular_resolution = pcl::deg2rad(0.5f);
    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image = *range_image_ptr;
    range_image.createFromPointCloud(cloud_velo, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     velo_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range,
                                     border_size);
//    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0,
                                                                                                    0, 255);
    viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
////    viewer.addCoordinateSystem (1.0f, "global");
////    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
////    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters();
    setViewerPose(viewer, range_image.getTransformationToWorldSystem());
//
//    // --------------------------
//    // -----Show range image-----
//    // --------------------------
//    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
//    range_image_widget.showRangeImage (range_image);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector;
    narf_keypoint_detector.setRangeImageBorderExtractor(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);
    std::cout << "Found " << keypoint_indices.points.size() << " key points.\n";

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &keypoints = *keypoints_ptr;
    keypoints.points.resize(keypoint_indices.points.size());
    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
        keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ>(keypoints_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    // ------------------------------------------------------
    // -----Extract NARF descriptors for interest points-----
    // ------------------------------------------------------
    std::vector<int> keypoint_indices2;
    keypoint_indices2.resize(keypoint_indices.points.size());
    for (unsigned int i = 0; i < keypoint_indices.size(); ++i) // This step is necessary to get the right vector type
        keypoint_indices2[i] = keypoint_indices.points[i];
    pcl::NarfDescriptor narf_descriptor(&range_image, &keypoint_indices2);
    narf_descriptor.getParameters().support_size = support_size;
    narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
    pcl::PointCloud<pcl::Narf36> narf_descriptors;
    narf_descriptor.compute(narf_descriptors);
    cout << "Extracted " << narf_descriptors.size() << " descriptors for "
         << keypoint_indices.points.size() << " keypoints.\n";


    // Camera point cloud
    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
//    float noise_level = 0.0;
//    float min_range = 0.0f;
//    int border_size = 1;
//    float angular_resolution = pcl::deg2rad (0.5f);
    boost::shared_ptr<pcl::RangeImage> cam_range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &cam_range_image = *cam_range_image_ptr;
    cam_range_image.createFromPointCloud(cloud_cam, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                         cam_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
//    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
        cam_range_image.setUnseenToMaxRange();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
//    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//    viewer.setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> cam_range_image_color_handler(
            cam_range_image_ptr, 0, 0, 0);
    viewer.addPointCloud(cam_range_image_ptr, cam_range_image_color_handler, "cam_range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cam_range image");
//    viewer.addCoordinateSystem (1.0f, "global");
//    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
//    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters();
    setViewerPose(viewer, cam_range_image.getTransformationToWorldSystem());

    // --------------------------
    // -----Show range image-----
    // --------------------------
//    pcl::visualization::RangeImageVisualizer cam_range_image_widget ("Cam Range image");
//    cam_range_image_widget.showRangeImage (cam_range_image);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor cam_range_image_border_extractor;
    pcl::NarfKeypoint cam_narf_keypoint_detector;
    cam_narf_keypoint_detector.setRangeImageBorderExtractor(&cam_range_image_border_extractor);
    cam_narf_keypoint_detector.setRangeImage(&cam_range_image);
    cam_narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> cam_keypoint_indices;
    cam_narf_keypoint_detector.compute(cam_keypoint_indices);
    std::cout << "Found " << cam_keypoint_indices.points.size() << " key points.\n";

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &cam_keypoints = *cam_keypoints_ptr;
    cam_keypoints.points.resize(cam_keypoint_indices.points.size());
    for (size_t i = 0; i < cam_keypoint_indices.points.size(); ++i)
        cam_keypoints.points[i].getVector3fMap() = cam_range_image.points[cam_keypoint_indices.points[i]].getVector3fMap();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cam_keypoints_color_handler(cam_keypoints_ptr, 255,
                                                                                                0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cam_keypoints_ptr, cam_keypoints_color_handler, "cam_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "cam_keypoints");

    // ------------------------------------------------------
    // -----Extract NARF descriptors for interest points-----
    // ------------------------------------------------------
    std::vector<int> cam_keypoint_indices2;
    cam_keypoint_indices2.resize(cam_keypoint_indices.points.size());
    for (unsigned int i = 0;
         i < cam_keypoint_indices.size(); ++i) // This step is necessary to get the right vector type
        cam_keypoint_indices2[i] = cam_keypoint_indices.points[i];
    pcl::NarfDescriptor cam_narf_descriptor(&cam_range_image, &cam_keypoint_indices2);
    cam_narf_descriptor.getParameters().support_size = support_size;
    cam_narf_descriptor.getParameters().rotation_invariant = rotation_invariant;
    pcl::PointCloud<pcl::Narf36> cam_narf_descriptors;
    cam_narf_descriptor.compute(cam_narf_descriptors);
    cout << "Extracted " << cam_narf_descriptors.size() << " descriptors for "
         << cam_keypoint_indices.points.size() << " keypoints.\n";

//    // ----------------------------------------------------
//    // -----Match NARF descriptors and display matches-----
//    // ----------------------------------------------------
//    std::vector<int> match_inds;
//    match_inds = matchNarfDescriptors(narf_descriptors, cam_narf_descriptors);
//    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
//    {
//        if (match_inds[i] < cam_narf_descriptors.points.size()) {
//            pcl::PointXYZ velo_pt(narf_descriptors.points[i].x, narf_descriptors.points[i].y, narf_descriptors.points[i].z);
//            pcl::PointXYZ cam_pt(cam_narf_descriptors.points[match_inds[i]].x, cam_narf_descriptors.points[match_inds[i]].y, cam_narf_descriptors.points[match_inds[i]].z);
//            if (pcl::euclideanDistance(velo_pt, cam_pt) < 5.0) {
//                viewer.addLine(velo_pt, cam_pt, "line" + std::to_string(i), 0);
//                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1,
//                                                   "line" + std::to_string(i), 0);
//                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
//                                                   0, 0, 0, "line" + std::to_string(i), 0);
//            }
//
//        }
//    }

    pcl::PointCloud<pcl::PointXYZ> icp_keypoints_orig;
    pcl::PointCloud<pcl::PointXYZ> icp_keypoints;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i < keypoints_ptr->points.size(); i++)
    {
        pcl::PointXYZ pt;
        pt.x = keypoints_ptr->points[i].x;
        pt.y = keypoints_ptr->points[i].y;
        pt.z = keypoints_ptr->points[i].z;

        icp_keypoints_orig.push_back(pt);
    }


    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();



    transform_1 (0,0) =  9.991241e-01;
    transform_1 (0,1) = 1.328777e-02;
    transform_1 (0,2) = -3.967996e-02;
//    transform_1 (0,3) = -9.609163e-01;
    transform_1 (0,3) = 9.609163e-01;
    transform_1 (1,0) =  -1.412604e-02;
    transform_1 (1,1) = 9.996813e-01;
    transform_1 (1,2) = -2.092052e-02;
//    transform_1 (1,3) = -5.783595e-01;
    transform_1 (1,3) = 5.783595e-01;
    transform_1 (2,0) =  3.938933e-02;
    transform_1 (2,1) = 2.146271e-02;
    transform_1 (2,2) = 9.989934e-01;
//    transform_1 (2,3) = 1.726896e+01;
    transform_1 (2,3) = -1.726896e+01;

    //    (row, column)


    // Print the transformation
    printf ("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;
    pcl::transformPointCloud(icp_keypoints_orig, icp_keypoints, transform_1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_keypoints_ptr(&icp_keypoints);

    for (int i = 0; i < 50; ++i) {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(icp_keypoints_ptr);
        icp.setInputTarget(cam_keypoints_ptr);
        icp.setMaxCorrespondenceDistance (1.0);
        icp.setRANSACOutlierRejectionThreshold (1.0);
        icp.setTransformationEpsilon(1e-13);
        icp.setEuclideanFitnessEpsilon(1);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        *icp_keypoints_ptr = Final;
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                  icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }

//    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_keypoints_ptr(*keypoints_ptr);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_keypoints_color_handler(icp_keypoints_ptr, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZ>(icp_keypoints_ptr, icp_keypoints_color_handler, "icp_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "icp_keypoints");

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

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
    viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                              look_at_vector[0], look_at_vector[1], look_at_vector[2],
                              up_vector[0], up_vector[1], up_vector[2]);
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
            cout << desc << endl;
        }
    }

    return ind;
}