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

#include "load_point_cloud.h"
#include "keypoint_compare.h"

using namespace std;

int main(int argc, char** argv) {

    string data_path = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/";
    string filename2 = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/000001.bin";
    string calib_file = "/home/boroson/data/kitti/dataset/sequences/00/calib.txt";
    string poses_file = "/home/boroson/data/kitti/dataset/poses/00.txt";

    vector<Eigen::Matrix4f> poses = loadPoses(poses_file);
    int num_poses = poses.size();

    for (int i = 0; i < num_poses; ++i)
    {

        cout << "loading velodyne frame " << i << " from file" << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr velo_frame_in_cam0_frame_ptr = loadVelFrameInCam0Ref(i);

        cout << "loading stereo frame " << i << " from file" << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cam_frame_in_cam0_frame_ptr = loadCamFrameInCam0Ref(i);

        string filename = "/home/boroson/data/kitti/features/harris/";
        filename.append(to_string(i));
        filename.append(".txt");

        compare_harris_keypts(velo_frame_in_cam0_frame_ptr, cam_frame_in_cam0_frame_ptr, filename);
    }

    return 0;
}

