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

#include "matrix.h"

using namespace std;
using namespace octomap;

void read_scan(Pointcloud *pc, string filename);

vector<Matrix> loadPoses(string file_name);

pose6d transformToPose(Matrix t);

//void read_calib(cv::Mat& cal, string filename);

int main(int argc, char** argv) {

    cout << "reading velodyne file" << endl;

    OcTree tree (0.1);  // create empty tree with resolution 0.1

    string data_path = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/";
    string filename2 = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/000001.bin";
    string calib_file = "/home/boroson/data/kitti/dataset/sequences/00/calib.txt";
    string poses_file = "/home/boroson/data/kitti/dataset/poses/00.txt";

    vector<Matrix> poses = loadPoses(poses_file);
    int num_poses = poses.size();

    Pointcloud pc;
    point3d origin(0.0, 0.0, 0.0);
    // camera frame origin, in velodyne frame
    // From KITTI setup page, not calib file
    point3d cam_origin(0.27, 0.0, -0.08);
    // vector from initial position to frame 1 position in velodyne frame
    point3d t1(8.586941e-01, 4.690294e-02, 2.839928e-02);
    pose6d f1(t1.x(), t1.y(), t1.z(), 0.0, 0.0, 0.0);

    // Camera calibration parameters
    // From calib file, but hardcoded here, this is wrong and should be fixed
    float fx = 7.18856e+02;
    float fy = 7.18856e+02;
    float cx = 6.071928e+02;
    float cy = 1.852157e+02;
    int imsizex = 1241;
    int imsizey = 376;

    cout << "making tree" << endl;

    for (int i = 20; i < 21; ++i)
    {
        char filename[10];
        sprintf(filename, "%06d.bin", i);
        string scan_file = data_path + string(filename);
        cout << "Read file " << filename << endl;

        read_scan(&pc, scan_file);
        Matrix trans = poses[i];
        pose6d f = transformToPose(trans);
        // TODO: check sensor_origin argument
        tree.insertPointCloud(pc, origin, f);
        pc.clear();
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

    tree.writeBinary("velo_tree_frame20.bt");

//    ofstream outfile;
//    outfile.open ("nodes.txt");
//
//    for(OcTree::tree_iterator it = tree.begin_tree(),
//                end=tree.end_tree(); it!= end; ++it)
//    {
//        //manipulate node, e.g.:
//        outfile << "Node center: " << it.getCoordinate() << endl;
//        outfile << "Node size: " << it.getSize() << endl;
//        outfile << "Node value: " << it->getValue() << endl;
////        v=v+(pow(it.getSize(),3));
//    }
//
//    outfile.close();

//    std::cout<<"VOLUME::::"<<v<<endl;
//    exit(0);

    point3d direction;
    point3d ray_end;

    cv::Mat inv_depths = cv::Mat::zeros(imsizey, imsizex, CV_32F);
    cv::Mat depth_img = cv::Mat::zeros(imsizey, imsizex, CV_8UC1);

//    uint8_t img[200][200]

    for (int yi = 0; yi < imsizey; ++yi) {
        for (int xi = 0; xi < imsizex; ++xi) {
//  for(float z = 0; z <= 0.25; z += 0.125){
            direction = point3d(1, -(xi - cx)/fx, -(yi - cy)/fy);
//            cout << endl;
//            cout << "casting ray from " << origin  << " in the " << direction << " direction"<< endl;
            bool success = tree.castRay(cam_origin, direction, ray_end);

            if (success) {
                inv_depths.at<float>(yi,xi) = 1/ray_end.x();
//                depth_img.at<uchar>(yi,xi) = uint8_t(ray_end.x()*10);
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
    cv::imshow("window", depth_img);

//    cv::imwrite("depth_image_velo_000000.png", depth_img);

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
    trans.x() = t.val[2][3];
    trans.y() = -t.val[0][3];
    trans.z() = -t.val[1][3];

    // I'm not sure why quaternion type doesn't have function to convert from rotation matrix. I'm actually
    // going to convert to Euler angles to be more human-readable. Eventually this should be replaced with
    // conversion directly to quaternion. (Using rotation matrix defined in Quaternion init from Euler angles.)
    // still trying to figure out angles. best guess so far is
    // rot.y() = -asinf(t.val[2][1]);
    // (approx 5 million nodes)
    rot.y() = -asinf(t.val[2][1]);
    rot.x() = -asinf(t.val[0][1]/cos(rot.y()));
    rot.z() = asinf(t.val[2][0]/cos(rot.y()));

    pose6d pose(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z());

    return pose;
}
