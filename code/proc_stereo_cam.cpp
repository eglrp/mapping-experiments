//
// Created by boroson on 3/27/18.
//

/*
 * Function to read images from two stereo cameras, compute depths, and create octomap
 */

#include <vector>
#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap/Pointcloud.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "matrix.h"

using namespace std;
using namespace octomap;


vector<Matrix> loadPoses(string file_name);

pose6d transformToPose(Matrix t);

//void read_calib(cv::Mat& cal, string filename);

int main(int argc, char** argv) {

    cout << "reading velodyne file" << endl;

    OcTree tree (0.1);  // create empty tree with resolution 0.1

    string data_path_0 = "/home/boroson/data/kitti/dataset/sequences/00/image_0/";
    string data_path_1 = "/home/boroson/data/kitti/dataset/sequences/00/image_1/";
    string calib_file = "/home/boroson/data/kitti/dataset/sequences/00/calib.txt";
    string poses_file = "/home/boroson/data/kitti/dataset/poses/00.txt";

    vector<Matrix> poses = loadPoses(poses_file);
    int num_poses = poses.size();

    Pointcloud pc;
    point3d origin(0.0, 0.0, 0.0);
    // camera frame origin, in velodyne frame
    // From KITTI setup page, not calib file
    // vector from initial position to frame 1 position in velodyne frame
    point3d t1(8.586941e-01, 4.690294e-02, 2.839928e-02);
    pose6d f1(t1.x(), t1.y(), t1.z(), 0.0, 0.0, 0.0);

    // Camera calibration parameters
    // From calib file, but hardcoded here, this is wrong and should be fixed
    double fx = 7.18856e+02;
    double fy = 7.18856e+02;
    double cx = 6.071928e+02;
    double cy = 1.852157e+02;
    int imsizex = 1241;
    int imsizey = 376;

//    // Camera matrices - distortion has already been removed
//    cv::Mat cam0mat = cv::Mat::zeros(3, 3, CV_64F);
//    cv::Mat cam1mat = cv::Mat::zeros(3, 3, CV_64F);
//    cv::Mat cam0dist = cv::Mat::zeros(1, 5, CV_64F);
//    cv::Mat cam1dist = cv::Mat::zeros(1, 5, CV_64F);
//
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


    cout << "making tree" << endl;

    for (int k = 80; k < 140; ++k)
    {
        char filename[10];
        sprintf(filename, "%06d.png", k);
        string im0_file = data_path_0 + string(filename);
        string im1_file = data_path_1 + string(filename);
        cout << "Read file " << im0_file << endl;

        cv::Mat im0 = cv::imread(im0_file);
        cv::Mat im1 = cv::imread(im1_file);

        // Camera matrices - distortion has already been removed
        cv::Mat cam0mat = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat cam1mat = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat cam0dist = cv::Mat::zeros(1, 5, CV_64F);
        cv::Mat cam1dist = cv::Mat::zeros(1, 5, CV_64F);

        cam0mat.at<double>(0,0) = fx;
        cam0mat.at<double>(1,1) = fy;
        cam0mat.at<double>(0,2) = cx;
        cam0mat.at<double>(1,2) = cy;
        cam0mat.at<double>(2,2) =  1;

        cam1mat.at<double>(0,0) = fx;
        cam1mat.at<double>(1,1) = fy;
        cam1mat.at<double>(0,2) = cx;
        cam1mat.at<double>(1,2) = cy;
        cam1mat.at<double>(2,2) =  1;

        cv::Size img_size = im0.size();

        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat T = cv::Mat::zeros(3, 1, CV_64F);
        T.at<double>(0,0) = -0.54*15; //-3.8614e+02 / 10;
//        T.at<double>(0,0) = -3.8614e+02 / 10;
        cv::Mat R1, P1, R2, P2;

        cv::Mat Q;
        cv::Rect roi1, roi2;

        cout << "Computing stereo camera transforms" << endl;

        cv::stereoRectify( cam0mat, cam0dist, cam1mat, cam1dist, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );

        cout << P1 << endl;
        cout << P2 << endl;
        cout << Q << endl;

        cv::Mat map11, map12, map21, map22;
        cv::initUndistortRectifyMap(cam0mat, cam0dist, R1, P1, img_size, CV_16SC2, map11, map12);
        cv::initUndistortRectifyMap(cam1mat, cam1dist, R2, P2, img_size, CV_16SC2, map21, map22);

        cv::Mat img1r, img2r;
        cv::remap(im0, img1r, map11, map12, cv::INTER_LINEAR);
        cv::remap(im1, img2r, map21, map22, cv::INTER_LINEAR);

        im0 = img1r;
        im1 = img2r;

        cout << "Setting up block matcher" << endl;

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
        cout << "Computing disparities" << endl;
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

        Matrix trans = poses[k];
        pose6d f = transformToPose(trans);
        // TODO: check sensor_origin argument
//        tree.insertPointCloud(pc, origin, f);

        for (int i = 0; i < xyzarr[0].rows; ++i) {
            for (int j = 0; j < xyzarr[0].cols; ++j) {
                if (xyzarr[2].at<float>(i,j) < 1000) {
                    point3d pt(xyzarr[0].at<float>(i, j), xyzarr[1].at<float>(i, j), xyzarr[2].at<float>(i, j));
                    tree.insertRay(f.trans(), f.transform(pt));
                }
            }
        }

    }

//    read_scan(&pc, data_path);



//    // insert points into octomap
//    for (Pointcloud::iterator pc_iter = pc.begin(); pc_iter < pc.end(); ++pc_iter) {
//        point3d point = *pc_iter;
//        tree.updateNode(point, true);
//    }


//    for ()
//    tree.insertPointCloud(pc, origin);
//    pc.clear();

//    read_scan(&pc, filename2);
//    tree.insertPointCloud(pc, t1, f1);

    cout << "writing tree out" << endl;

    tree.writeBinary("cam_tree4.bt");

    point3d direction;
    point3d ray_end;

    cv::Mat inv_depths = cv::Mat::zeros(imsizey, imsizex, CV_32F);
    cv::Mat depth_img = cv::Mat::zeros(imsizey, imsizex, CV_8UC1);

//    uint8_t img[200][200]

    for (int yi = 0; yi < imsizey; ++yi) {
        for (int xi = 0; xi < imsizex; ++xi) {
//  for(float z = 0; z <= 0.25; z += 0.125){
            direction = point3d(float((xi - cx)/fx), float((yi - cy)/fy), 1);
//            cout << endl;
//            cout << "casting ray from " << origin  << " in the " << direction << " direction"<< endl;
            bool success = tree.castRay(origin, direction, ray_end);

            if (success) {
                inv_depths.at<float>(yi,xi) = 1/ray_end.z();
        cout << "entrance point is " << ray_end << endl;
            }
        }
    }

//    cout << inv_depths << endl;

    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;

    cv::minMaxLoc( inv_depths, &minVal, &maxVal, &minLoc, &maxLoc );

//    depth_img = inv_depths/ maxVal;

    inv_depths.convertTo(depth_img, CV_8UC1, 255/maxVal);

//    cout << depth_img << endl;

    cv::imshow("window", depth_img);

//    cv::imwrite("depth_image_stereo_5_000000.png", depth_img);

    cv::waitKey(0);

    return 0;
}

void read_scan(Pointcloud *pc, string filename)
{
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
//  float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen (filename.c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;
    for (int32_t i=0; i<num; i++) {
        pc->push_back(point3d(*px,*py,*pz));//,*pr));
        px+=4; py+=4; pz+=4; //pr+=4;
    }
    fclose(stream);
}

//void read_calib(cv::Mat& cal, string filename)
//{
//    string line;
//    cal = cv::Mat::zeros(3,4,CV_32F);
//    cv::Mat p = cv::Mat::zeros(3,4,CV_32F);
//    cv::Mat t = cv::Mat::zeros(3,4,CV_32F);
//
//    FILE *cal_file = fopen(filename.c_str(), "r");
//    float p00, p01, p02, p03;
//    float p10, p11, p12, p13;
//    float p20, p21, p22, p23;
//    float t00, t01, t02, t03;
//    float t10, t11, t12, t13;
//    float t20, t21, t22, t23;
//
//    fscanf(cal_file, "P0: %e %e %e %e %e %e %e %e %e %e %e %e\n",
//           &p00, &p01, &p02, &p03, &p10, &p11, &p12, &p13, &p20, &p21, &p22, &p23);
//    fscanf(cal_file, "P1: %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e\n");
//    fscanf(cal_file, "P2: %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e\n");
//    fscanf(cal_file, "P3: %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e %*e\n");
//    fscanf(cal_file, "Tr: %e %e %e %e %e %e %e %e %e %e %e %e\n",
//           &t00, &t01, &t02, &t03, &t10, &t11, &t12, &t13, &t20, &t21, &t22, &t23);
//
//}

vector<Matrix> loadPoses(string file_name)
{
    vector<Matrix> poses;
    FILE *fp = fopen(file_name.c_str(),"r");
    if (!fp)
        return poses;
    while (!feof(fp)) {
        Matrix P = Matrix::eye(4);
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P.val[0][0], &P.val[0][1], &P.val[0][2], &P.val[0][3],
                   &P.val[1][0], &P.val[1][1], &P.val[1][2], &P.val[1][3],
                   &P.val[2][0], &P.val[2][1], &P.val[2][2], &P.val[2][3] )==12) {
            poses.push_back(P);
        }
    }
    fclose(fp);
    return poses;
}

pose6d transformToPose(Matrix t)
{
    point3d trans, rot;
//    // from lidar
//    trans.x() = t.val[2][3];
//    trans.y() = -t.val[0][3];
//    trans.z() = -t.val[1][3];

    // from camera?
    trans.z() = t.val[2][3];
    trans.x() = t.val[0][3];
    trans.y() = t.val[1][3];

    // I'm not sure why quaternion type doesn't have function to convert from rotation matrix. I'm actually
    // going to convert to Euler angles to be more human-readable. Eventually this should be replaced with
    // conversion directly to quaternion. (Using rotation matrix defined in Quaternion init from Euler angles.)
    // still trying to figure out angles. best guess so far is
    // rot.y() = -asinf(t.val[2][1]);
    // (approx 5 million nodes)
//  //   from lidar
//    rot.y() = -asinf(t.val[2][1]);
//    rot.x() = -asinf(t.val[0][1]/cos(rot.y()));
//    rot.z() = asinf(t.val[2][0]/cos(rot.y()));

    //   from camera?
    rot.x() = asinf(t.val[2][1]);
    rot.z() = -asinf(t.val[0][1]/cos(rot.y()));
    rot.y() = -asinf(t.val[2][0]/cos(rot.y()));

    pose6d pose(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z());

    return pose;
}
