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
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "matrix.h"

using namespace std;
using namespace octomap;

void read_scan(Pointcloud *pc, string filename);

vector<Matrix> loadPoses(string file_name);

pose6d transformToPose(Matrix t);

//void read_calib(cv::Mat& cal, string filename);

int main(int argc, char** argv) {

    cout << "loading velodyne octomap from file" << endl;

    OcTree velo_tree ("velo_tree_frame0.bt");  // load from file

    cout << "loading stereo image octomap from file" << endl;

    OcTree cam_tree ("velo_tree_frame20.bt");  // load from file

    string data_path = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/";
    string filename2 = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/000001.bin";
    string calib_file = "/home/boroson/data/kitti/dataset/sequences/00/calib.txt";
    string poses_file = "/home/boroson/data/kitti/dataset/poses/00.txt";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_velo (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam (new pcl::PointCloud<pcl::PointXYZ>);

    for(OcTree::leaf_iterator it = velo_tree.begin_leafs(),
                end=velo_tree.end_leafs(); it!= end; ++it)
    {
        if (it->getValue() > 0.0) {
            cloud_velo->push_back(
                    pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
        }
    }

    // Estimate normals on velo dataset (from http://pointclouds.org/documentation/tutorials/normal_estimation.php)
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_velo;
    ne_velo.setInputCloud (cloud_velo);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne_velo.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_velo_normals (new pcl::PointCloud<pcl::Normal>);
    ne_velo.setRadiusSearch (0.10); //meters

    // Compute the features
    ne_velo.compute (*cloud_velo_normals);

    for(OcTree::leaf_iterator it = cam_tree.begin_leafs(),
                end=cam_tree.end_leafs(); it!= end; ++it)
    {
        if (it->getValue() > 0.0) {
            cloud_cam->push_back(pcl::PointXYZ(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()));
        }
    }

    // Estimate normals on cam dataset (from http://pointclouds.org/documentation/tutorials/normal_estimation.php)
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_cam;
    ne_cam.setInputCloud (cloud_cam);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cam (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne_cam.setSearchMethod (tree_cam);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_cam_normals (new pcl::PointCloud<pcl::Normal>);
    ne_cam.setRadiusSearch (0.10); //meters

    // Compute the features
    ne_cam.compute (*cloud_cam_normals);

    std::cout << "Saved " << cloud_cam->points.size () << " data points from camera"
              << std::endl;
    std::cout << "Saved " << cloud_velo->points.size () << " data points from velodyne"
              << std::endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_velo_pt_norm (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cam_pt_norm (new pcl::PointCloud<pcl::PointNormal>);

    for(int i = 0; i < cloud_velo_normals->points.size(); i++)
    {
        pcl::PointNormal pt;
        pt.x = cloud_velo->points[i].x;
        pt.y = cloud_velo->points[i].y;
        pt.z = cloud_velo->points[i].z;

        pt.curvature = cloud_velo_normals->points[i].curvature;
        pt.normal_x = cloud_velo_normals->points[i].normal_x;
        pt.normal_y = cloud_velo_normals->points[i].normal_y;
        pt.normal_z = cloud_velo_normals->points[i].normal_z;
        cloud_velo_pt_norm->push_back(pt);
    }

    for(int i = 0; i < cloud_cam_normals->points.size(); i++)
    {
        pcl::PointNormal pt;
        pt.x = cloud_cam->points[i].x;
        pt.y = cloud_cam->points[i].y;
        pt.z = cloud_cam->points[i].z;

        pt.curvature = cloud_cam_normals->points[i].curvature;
        pt.normal_x = cloud_cam_normals->points[i].normal_x;
        pt.normal_y = cloud_cam_normals->points[i].normal_y;
        pt.normal_z = cloud_cam_normals->points[i].normal_z;
        cloud_cam_pt_norm->push_back(pt);
    }

    //remove NAN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNNormalsFromPointCloud(*cloud_velo_pt_norm,*cloud_velo_pt_norm, indices);
    pcl::removeNaNNormalsFromPointCloud(*cloud_cam_pt_norm,*cloud_cam_pt_norm, indices);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source (cloud_velo_pt_norm);

    for (int i = 0; i < 50; ++i) {
        pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setInputSource(cloud_source);
        icp.setInputTarget(cloud_cam_pt_norm);
        icp.setTransformationEpsilon(1e-13);
        icp.setEuclideanFitnessEpsilon(1);
        pcl::PointCloud<pcl::PointNormal> Final;
        icp.align(Final);
        *cloud_source = Final;
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                  icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }


    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0,
                                                                                                    0, 255);
    viewer.addPointCloud(cloud_velo_pt_norm, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
////    viewer.addCoordinateSystem (1.0f, "global");
////    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (&cloud_velo, 150, 150, 150);
////    viewer.addPointCloud (&cloud_velo, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters();
    setViewerPose(viewer, range_image.getTransformationToWorldSystem());

    return 0;
}

