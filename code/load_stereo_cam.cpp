//
// Created by lboroson on 6/28/18.
//

#include <string>
#include <vector>

#include "load_point_cloud.h"
#include <pcl/common/transforms.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

//vector<Eigen::Matrix4f> load_cam_poses(string file_name)
//{
//    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
//    double t0, t1, t2;
//    vector<Eigen::Matrix4f> poses;
//    FILE *fp = fopen(file_name.c_str(),"r");
//    if (!fp)
//        return poses;
//    while (!feof(fp)) {
//        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
//                   &r00, &r01, &r02, &t0,
//                   &r10, &r11, &r12, &t1,
//                   &r20, &r21, &r22, &t2 )==12) {
//            Eigen::Matrix4f Tcami2cam0;
//            Tcami2cam0 << r00, r01, r02, t0,
//                          r10, r11, r12, t1,
//                          r20, r21, r22, t2,
//                            0,   0,   0,  1;
//            poses.push_back(Tcami2cam0);
//        }
//    }
//    fclose(fp);
//    return poses;
//}

//Eigen::Matrix4f loadCam12Cam0(string file_name)
//{
//    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
//    double t0, t1, t2;
//    char matname[8];
//    Eigen::Matrix4f Tvel2cam = Eigen::Matrix4f::Identity();
//    FILE *fp = fopen(file_name.c_str(),"r");
//    if (!fp)
//        return Tvel2cam;
//    while (!feof(fp)) {
//        if (fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
//                   matname, &r00, &r01, &r02, &t0,
//                   &r10, &r11, &r12, &t1,
//                   &r20, &r21, &r22, &t2 )==13) {
//            if ((matname[0] == 'T') && (matname[1] == 'r'))
//            {
//                Tvel2cam << r00, r01, r02, t0,
//                        r10, r11, r12, t1,
//                        r20, r21, r22, t2,
//                        0, 0, 0, 1;
//            }
//        }
//    }
//    fclose(fp);
//    return Tvel2cam;
//}

cv::Mat load_proj_mat(string file_name)
{
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    double t0, t1, t2;
    char matname[8];
    cv::Mat cam0mat = cv::Mat::zeros(3, 4, CV_64F);
    FILE *fp = fopen(file_name.c_str(),"r");
    if (!fp)
        return cam0mat;
    while (!feof(fp)) {
        if (fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   matname, &cam0mat.at<double>(0,0), &cam0mat.at<double>(0,1),  &cam0mat.at<double>(0,2),  &cam0mat.at<double>(0,3),
                   &cam0mat.at<double>(1,0), &cam0mat.at<double>(1,1),  &cam0mat.at<double>(1,2),  &cam0mat.at<double>(1,3),
                   &cam0mat.at<double>(2,0), &cam0mat.at<double>(2,1),  &cam0mat.at<double>(2,2),  &cam0mat.at<double>(2,3) )==13) {
            if ((matname[0] == 'P') && (matname[1] == '1'))
            {
                break;
            }
        }
    }
    fclose(fp);
    return cam0mat;
}

//pose6d transformToPose(Matrix t)
//{
//    point3d trans, rot;
//    trans.x() = t.val[2][3];
//    trans.y() = -t.val[0][3];
//    trans.z() = -t.val[1][3];
//
//    // I'm not sure why quaternion type doesn't have function to convert from rotation matrix. I'm actually
//    // going to convert to Euler angles to be more human-readable. Eventually this should be replaced with
//    // conversion directly to quaternion. (Using rotation matrix defined in Quaternion init from Euler angles.)
//    // still trying to figure out angles. best guess so far is
//    // rot.y() = -asinf(t.val[2][1]);
//    // (approx 5 million nodes)
//    rot.y() = -asinf(t.val[2][1]);
//    rot.x() = -asinf(t.val[0][1]/cos(rot.y()));
//    rot.z() = asinf(t.val[2][0]/cos(rot.y()));
//
//    pose6d pose(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z());
//
//    return pose;
//}

void read_frame(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, cv::Mat p1_proj_mat, string im0_file, string im1_file)
{

    cv::Mat im0 = cv::imread(im0_file);
    cv::Mat im1 = cv::imread(im1_file);

    // Camera matrices - distortion has already been removed
    //TODO: Un-hardcode this
//    cv::Mat cam0mat = cv::Mat::zeros(3, 3, CV_64F);
//    cv::Mat cam1mat = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat cam0mat = p1_proj_mat( cv::Rect( 0, 0, 3, 3 ) );
    cv::Mat cam1mat = p1_proj_mat( cv::Rect( 0, 0, 3, 3 ) );
    cv::Mat cam0dist = cv::Mat::zeros(1, 5, CV_64F);
    cv::Mat cam1dist = cv::Mat::zeros(1, 5, CV_64F);

//    cam0mat.at<double>(0,0) = fx;
//    cam0mat.at<double>(1,1) = fy;
//    cam0mat.at<double>(0,2) = cx;
//    cam0mat.at<double>(1,2) = cy;
//    cam0mat.at<double>(2,2) =  1;
//
//    cam1mat.at<double>(0,0) = fx;
//    cam1mat.at<double>(1,1) = fy;
//    cam1mat.at<double>(0,2) = cx;
//    cam1mat.at<double>(1,2) = cy;
//    cam1mat.at<double>(2,2) =  1;

    cv::Size img_size = im0.size();

    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat T = p1_proj_mat( cv::Rect( 3, 0, 1, 3 ) );
    T = 0.022*T;
//    cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
//    T.at<double>(0,0) = -0.54*15; //-3.8614e+02 / 10;
//        T.at<double>(0,0) = -3.8614e+02 / 10;
    cv::Mat R1, P1, R2, P2;
//    cout << R << endl;
//    cout << T << endl;

    cv::Mat Q;
    cv::Rect roi1, roi2;

//    cout << "Computing stereo camera transforms" << endl;

    cv::stereoRectify( cam0mat, cam0dist, cam1mat, cam1dist, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );

//    cout << P1 << endl;
//    cout << P2 << endl;
//    cout << Q << endl;

    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(cam0mat, cam0dist, R1, P1, img_size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(cam1mat, cam1dist, R2, P2, img_size, CV_16SC2, map21, map22);

    cv::Mat img1r, img2r;
    cv::remap(im0, img1r, map11, map12, cv::INTER_LINEAR);
    cv::remap(im1, img2r, map21, map22, cv::INTER_LINEAR);

    im0 = img1r;
    im1 = img2r;

//    cout << "Setting up block matcher" << endl;

    int numberOfDisparities = ((img_size.width/8) + 15) & -16;

//        bm->setROI1(roi1);
//        bm->setROI2(roi2);
//        bm->setPreFilterCap(31);
//        bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
//        bm->setMinDisparity(0);
//        bm->setNumDisparities(numberOfDisparities);
//        bm->setTextureThreshold(10);
//        bm->setUniquenessRatio(15);
//        bm->setSpeckleWindowSize(100);
//        bm->setSpeckleRange(32);
//        bm->setDisp12MaxDiff(1);

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = im0.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
//        if(alg==STEREO_HH)
//            sgbm->setMode(StereoSGBM::MODE_HH);
//        else if(alg==STEREO_SGBM)
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
//        else if(alg==STEREO_3WAY)
//            sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);


    cv::Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

//        int64 t = getTickCount();
//        if( alg == STEREO_BM )
//            bm->compute(img1, img2, disp);
//        else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
//    cout << "Computing disparities" << endl;
    sgbm->compute(im0, im1, disp);
//        t = getTickCount() - t;
//        printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
//        if( alg != STEREO_VAR )
//            disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
//        else
    disp.convertTo(disp8, CV_8U);
//        if( !no_display )
//        {
//        cv::namedWindow("left", 1);
//        cv::imshow("left", im0);
//        cv::namedWindow("right", 1);
//        cv::imshow("right", im1);
//        cv::namedWindow("disparity", 0);
//        cv::imshow("disparity", disp8);
////            printf("press any key to continue...");
////            fflush(stdout);
//        cv::waitKey();
////            printf("\n");
////        }
//
//        if(!disparity_filename.empty())
//            imwrite(disparity_filename, disp8);

    //        if(!point_cloud_filename.empty())
//        {
    printf("computing the point cloud...");
    cv::Mat xyz;
    cv::reprojectImageTo3D(disp, xyz, Q, true);
//            saveXYZ(point_cloud_filename.c_str(), xyz);
//            printf("\n");
//        }
    cv::Mat xyzarr[3];
    cv::split(xyz, xyzarr);

//    Matrix trans = poses[k];
//    pose6d f = transformToPose(trans);
//    // TODO: check sensor_origin argument
////        tree.insertPointCloud(pc, origin, f);

    for (int i = 0; i < xyzarr[0].rows; i = i+5) {
        for (int j = 0; j < xyzarr[0].cols; j = j+5) {
            if (xyzarr[2].at<float>(i, j) < 1000) {
                pc->push_back(
                        pcl::PointXYZ(xyzarr[0].at<float>(i, j), xyzarr[1].at<float>(i, j), xyzarr[2].at<float>(i, j)));
            }
        }
    }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr loadCamFrameInCam0Ref(int frame_number, int seq_number)
{
    char seq_chars[2];
    sprintf(seq_chars, "%02d", seq_number);
    string data_path = "/home/boroson/data/kitti/dataset/sequences/" + string(seq_chars) + "/";

    vector<Eigen::Matrix4f> poses = loadPoses("/home/boroson/data/kitti/dataset/poses/" + string(seq_chars) + ".txt");
    int num_poses = poses.size();
//    Eigen::Matrix4f Tvel2cam = loadVel2Cam("/home/boroson/data/kitti/dataset/sequences/00/calib.txt");
    cv::Mat p1_proj = load_proj_mat("/home/boroson/data/kitti/dataset/sequences/" + string(seq_chars) + "/calib.txt");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_cam_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_cam0_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    char filename[10];
    sprintf(filename, "%06d.png", frame_number);
    string im0_file = data_path + "image_0/" + string(filename);
    string im1_file = data_path + "image_1/" + string(filename);
    cout << "Read file " << filename << endl;

    read_frame(cam_cloud_cam_frame_ptr, p1_proj, im0_file, im1_file);
    // poses[frame_number] = TCami2Cam0
    pcl::transformPointCloud(*cam_cloud_cam_frame_ptr, *cam_cloud_cam0_frame_ptr, poses[frame_number]);

    return cam_cloud_cam0_frame_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCamFrameInCamiRef(int frame_number, int seq_number)
{
    char seq_chars[2];
    sprintf(seq_chars, "%02d", seq_number);
    string data_path = "/home/boroson/data/kitti/dataset/sequences/" + string(seq_chars) + "/";

    vector<Eigen::Matrix4f> poses = loadPoses("/home/boroson/data/kitti/dataset/poses/" + string(seq_chars) + ".txt");
    int num_poses = poses.size();
//    Eigen::Matrix4f Tvel2cam = loadVel2Cam("/home/boroson/data/kitti/dataset/sequences/00/calib.txt");
    cv::Mat p1_proj = load_proj_mat("/home/boroson/data/kitti/dataset/sequences/" + string(seq_chars) + "/calib.txt");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_cam_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    char filename[10];
    sprintf(filename, "%06d.png", frame_number);
    string im0_file = data_path + "image_0/" + string(filename);
    string im1_file = data_path + "image_1/" + string(filename);
    cout << "Read file " << im0_file << endl;

    read_frame(cam_cloud_cam_frame_ptr, p1_proj, im0_file, im1_file);
    // poses[frame_number] = TCami2Cam0

//    cout << "True rotation from this frame to Cam i frame: \n" << poses[frame_number] << endl;

    return cam_cloud_cam_frame_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCamFrameInCamjRef(int frame_number, int seq_number)
{
    char seq_chars[2];
    sprintf(seq_chars, "%02d", seq_number);
    string data_path = "/home/boroson/data/kitti/dataset/sequences/" + string(seq_chars) + "/";

    vector<Eigen::Matrix4f> poses = loadPoses("/home/boroson/data/kitti/dataset/poses/" + string(seq_chars) + ".txt");
    int num_poses = poses.size();
//    Eigen::Matrix4f Tvel2cam = loadVel2Cam("/home/boroson/data/kitti/dataset/sequences/00/calib.txt");
    cv::Mat p1_proj = load_proj_mat("/home/boroson/data/kitti/dataset/sequences/" + string(seq_chars) + "/calib.txt");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud_cam_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    char filename[10];
    sprintf(filename, "%06d.png", frame_number);
    string im0_file = data_path + "image_0/" + string(filename);
    string im1_file = data_path + "image_1/" + string(filename);
    cout << "Read file " << im0_file << endl;

    read_frame(cam_cloud_cam_frame_ptr, p1_proj, im0_file, im1_file);
    // poses[frame_number] = TCami2Cam0

//    cout << "True rotation from this frame to Cam i frame: \n" << poses[frame_number] << endl;

    return cam_cloud_cam_frame_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadOverlapRegion(int vel_frame_number, int cam_frame_number, int seq_number)
{
    char seq_chars[2];
    sprintf(seq_chars, "%02d", seq_number);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_limits_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cv::Mat p1_proj = load_proj_mat("/home/boroson/data/kitti/dataset/sequences/" + string(seq_chars) + "/calib.txt");

    vector<Eigen::Matrix4f> poses = loadPoses("/home/boroson/data/kitti/dataset/poses/" + string(seq_chars) + ".txt");

    double fx = p1_proj.at<double>(0,0);
    double fy = p1_proj.at<double>(1,1);
    double cx = p1_proj.at<double>(0,2);
    double cy = p1_proj.at<double>(1,2);

    float max_dist = 120.0; //from Velodyne datasheet
    float img_size_x = 1392.0;
    float img_size_y = 512.0;

    if (vel_frame_number != cam_frame_number) {
        float zdiff = poses[vel_frame_number](2, 3) - poses[cam_frame_number](2, 3);
        max_dist += zdiff;
    }
    hull_limits_ptr->push_back(
            pcl::PointXYZ((float) (0.0 - cx) * max_dist / fx, (float) (0.0 - cy) * max_dist / fy, max_dist));
    hull_limits_ptr->push_back(
            pcl::PointXYZ((float) (img_size_x - cx) * max_dist / fx, (float) (0.0 - cy) * max_dist / fy, max_dist));
    hull_limits_ptr->push_back(
            pcl::PointXYZ((float) (0.0 - cx) * max_dist / fx, (float) (img_size_y - cy) * max_dist / fy, max_dist));
    hull_limits_ptr->push_back(
            pcl::PointXYZ((float) (img_size_x - cx) * max_dist / fx, (float) (img_size_y - cy) * max_dist / fy,
                          max_dist));
    hull_limits_ptr->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));


//    pcl::ConvexHull<pcl::PointXYZ>::Ptr hull(new pcl::ConvexHull<pcl::PointXYZ>);
//    hull->reconstruct(*hull_limits_ptr);

    return hull_limits_ptr;
}


