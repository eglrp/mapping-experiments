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
//#include <pcl/keypoints/harris_3d.h>

#include "load_point_cloud.h"
#include "keypoint_compare.h"

using namespace std;

int main(int argc, char** argv) {

//    string data_path = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/";
//    string filename2 = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/000001.bin";
//    string calib_file = "/home/boroson/data/kitti/dataset/sequences/00/calib.txt";
    string poses_file = "/home/boroson/data/kitti/dataset/poses/00.txt";

    vector<Eigen::Matrix4f> poses = loadPoses(poses_file);
    int num_poses = poses.size();
    int traj = 0;

//    for (int i = 0; i < num_poses; ++i)
////    for (int i = 1270; i < num_poses; ++i)
//    {
//
//        cout << "loading velodyne frame " << i << " from file" << endl;
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame_in_cam0_frame_ptr = loadVelFrameInCamiRef(i, traj);
//
//        cout << "loading stereo frame " << i << " from file" << endl;
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cam_frame_in_cam0_frame_ptr = loadCamFrameInCamiRef(i, traj);
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_hull_ptr = loadOverlapRegion(i, i, traj);
//
////        string filename_keypt = "/home/boroson/data/kitti/features/narf/00_sameframe/";
////        filename_keypt.append(to_string(i));
////        filename_keypt.append("_keypt");
////        string filename_desc = "/home/boroson/data/kitti/features/narf/00_sameframe/";
////        filename_desc.append(to_string(i));
////        filename_desc.append("_desc");
////
////        compare_narf_features(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename_keypt, filename_desc); //, intersection_hull_ptr);
////
////        string filename = "/home/boroson/data/kitti/features/harris/00_sameframe/";
////        filename.append(to_string(i));
//
////        compare_harris_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//
////        string filename = "/home/boroson/data/kitti/features/iss/00_sameframe/";
////        filename.append(to_string(i));
////
////        compare_iss_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//
//        string filename = "/home/boroson/data/kitti/features/kpqas/00_sameframe/";
//        filename.append(to_string(i));
//
//        compare_kpqas_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//
//    }


//    poses_file = "/home/boroson/data/kitti/dataset/poses/02.txt";
//
//    poses = loadPoses(poses_file);
//    num_poses = poses.size();
//    traj = 2;
//
//    for (int i = 0; i < num_poses; ++i)
////    for (int i = 1173; i < num_poses; ++i)
//    {
//
//        cout << "loading velodyne frame " << i << " from file" << endl;
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame_in_cam0_frame_ptr = loadVelFrameInCamiRef(i, traj);
//
//        cout << "loading stereo frame " << i << " from file" << endl;
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cam_frame_in_cam0_frame_ptr = loadCamFrameInCamiRef(i, traj);
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_hull_ptr = loadOverlapRegion(i, i, traj);
////
////        string filename_keypt = "/home/boroson/data/kitti/features/narf/02_sameframe/";
////        filename_keypt.append(to_string(i));
////        filename_keypt.append("_keypt");
////        string filename_desc = "/home/boroson/data/kitti/features/narf/02_sameframe/";
////        filename_desc.append(to_string(i));
////        filename_desc.append("_desc");
////
////        compare_narf_features(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename_keypt, filename_desc); //, intersection_hull_ptr);
//
////        string filename = "/home/boroson/data/kitti/features/harris/02_sameframe/";
////        filename.append(to_string(i));
////
////        compare_harris_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//
////        string filename = "/home/boroson/data/kitti/features/iss/02_sameframe/";
////        filename.append(to_string(i));
////
////        compare_iss_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//
//        string filename = "/home/boroson/data/kitti/features/kpqas/02_sameframe/";
//        filename.append(to_string(i));
//
//        compare_kpqas_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//    }


//    poses_file = "/home/boroson/data/kitti/dataset/poses/06.txt";
//
//    poses = loadPoses(poses_file);
//    num_poses = poses.size();
//    traj = 6;
//
//    for (int i = 0; i < num_poses; ++i)
////    for (int i = 744; i < num_poses; ++i)
//    {
//
//        cout << "loading velodyne frame " << i << " from file" << endl;
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame_in_cam0_frame_ptr = loadVelFrameInCamiRef(i, traj);
//
//        cout << "loading stereo frame " << i << " from file" << endl;
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cam_frame_in_cam0_frame_ptr = loadCamFrameInCamiRef(i, traj);
//
//        pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_hull_ptr = loadOverlapRegion(i, i, traj);
//
////        string filename_keypt = "/home/boroson/data/kitti/features/narf/06_sameframe/";
////        filename_keypt.append(to_string(i));
////        filename_keypt.append("_keypt");
////        string filename_desc = "/home/boroson/data/kitti/features/narf/06_sameframe/";
////        filename_desc.append(to_string(i));
////        filename_desc.append("_desc");
////
////        compare_narf_features(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename_keypt, filename_desc); //, intersection_hull_ptr);
//
////        string filename = "/home/boroson/data/kitti/features/harris/06_sameframe/";
////        filename.append(to_string(i));
////
////        compare_harris_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//
////        string filename = "/home/boroson/data/kitti/features/iss/06_sameframe/";
////        filename.append(to_string(i));
////
////        compare_iss_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//
//        string filename = "/home/boroson/data/kitti/features/kpqas/06_sameframe/";
//        filename.append(to_string(i));
//
//        compare_kpqas_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);
//    }


    poses_file = "/home/boroson/data/kitti/dataset/poses/09.txt";

    poses = loadPoses(poses_file);
    num_poses = poses.size();
    traj = 9;

    for (int i = 0; i < num_poses; ++i)
//    for (int i = 1436; i < num_poses; ++i)
    {

        cout << "loading velodyne frame " << i << " from file" << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame_in_cam0_frame_ptr = loadVelFrameInCamiRef(i, traj);

        cout << "loading stereo frame " << i << " from file" << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cam_frame_in_cam0_frame_ptr = loadCamFrameInCamiRef(i, traj);

        pcl::PointCloud<pcl::PointXYZ>::Ptr intersection_hull_ptr = loadOverlapRegion(i, i, traj);

//        string filename_keypt = "/home/boroson/data/kitti/features/narf/09_sameframe/";
//        filename_keypt.append(to_string(i));
//        filename_keypt.append("_keypt");
//        string filename_desc = "/home/boroson/data/kitti/features/narf/09_sameframe/";
//        filename_desc.append(to_string(i));
//        filename_desc.append("_desc");
//
//        compare_narf_features(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename_keypt, filename_desc); //, intersection_hull_ptr);

//        string filename = "/home/boroson/data/kitti/features/harris/09_sameframe/";
//        filename.append(to_string(i));
//
//        compare_harris_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);

//        string filename = "/home/boroson/data/kitti/features/iss/09_sameframe/";
//        filename.append(to_string(i));
//
//        compare_iss_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);

        string filename = "/home/boroson/data/kitti/features/kpqas/09_sameframe/";
        filename.append(to_string(i));

        compare_kpqas_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename, intersection_hull_ptr);

    }

    return 0;
}

