//
// Created by lboroson on 6/28/18.
//

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifndef CODE_LOAD_STEREO_CAM_H
#define CODE_LOAD_STEREO_CAM_H

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCamFrameInCam0Ref(int frame_number, int seq_number);

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCamFrameInCamiRef(int frame_number, int seq_number);

#endif //CODE_LOAD_STEREO_CAM_H
