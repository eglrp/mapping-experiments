//
// Created by lboroson on 6/28/18.
//

#include <string>
#include <vector>

#include "load_velodyne.h"
#include <pcl/common/transforms.h>

using namespace std;

vector<Eigen::Matrix4f> loadPoses(string file_name)
{
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    double t0, t1, t2;
    vector<Eigen::Matrix4f> poses;
    FILE *fp = fopen(file_name.c_str(),"r");
    if (!fp)
        return poses;
    while (!feof(fp)) {
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &r00, &r01, &r02, &t0,
                   &r10, &r11, &r12, &t1,
                   &r20, &r21, &r22, &t2 )==12) {
            Eigen::Matrix4f Tcami2cam0;
            Tcami2cam0 << r00, r01, r02, t0,
                          r10, r11, r12, t1,
                          r20, r21, r22, t2,
                            0,   0,   0,  1;
            poses.push_back(Tcami2cam0);
        }
    }
    fclose(fp);
    return poses;
}

Eigen::Matrix4f loadVel2Cam(string file_name)
{
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    double t0, t1, t2;
    char matname[8];
    Eigen::Matrix4f Tvel2cam = Eigen::Matrix4f::Identity();
    FILE *fp = fopen(file_name.c_str(),"r");
    if (!fp)
        return Tvel2cam;
    while (!feof(fp)) {
        if (fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   matname, &r00, &r01, &r02, &t0,
                   &r10, &r11, &r12, &t1,
                   &r20, &r21, &r22, &t2 )==13) {
            if ((matname[0] == 'T') && (matname[1] == 'r'))
            {
                Tvel2cam << r00, r01, r02, t0,
                        r10, r11, r12, t1,
                        r20, r21, r22, t2,
                        0, 0, 0, 1;
            }
        }
    }
    fclose(fp);
    return Tvel2cam;
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

void read_scan(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, string filename)
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
        pc->push_back(pcl::PointXYZ(*px,*py,*pz));//,*pr));
        px+=4; py+=4; pz+=4; //pr+=4;
    }
    fclose(stream);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr loadVelFrameInCam0Ref(int frame_number)
{
    string data_path = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/";

    vector<Eigen::Matrix4f> poses = loadPoses("/home/boroson/data/kitti/dataset/poses/00.txt");
    int num_poses = poses.size();
    Eigen::Matrix4f Tvel2cam = loadVel2Cam("/home/boroson/data/kitti/dataset/sequences/00/calib.txt");

    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud_vel_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud_cam0_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    char filename[10];
    sprintf(filename, "%06d.bin", frame_number);
    string scan_file = data_path + string(filename);
    cout << "Read file " << filename << endl;

    read_scan(velo_cloud_vel_frame_ptr, scan_file);
    Eigen::Matrix4f Tveli2cam0;
    if (frame_number > 0)
        Tveli2cam0 = poses[frame_number] * Tvel2cam;
    else
        Tveli2cam0 = Tvel2cam;

    pcl::transformPointCloud(*velo_cloud_vel_frame_ptr, *velo_cloud_cam0_frame_ptr, Tveli2cam0);

    return velo_cloud_cam0_frame_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadVelFrameInCamiRef(int frame_number)
{
    string data_path = "/home/boroson/data/kitti/dataset/sequences/00/velodyne/";

    vector<Eigen::Matrix4f> poses = loadPoses("/home/boroson/data/kitti/dataset/poses/00.txt");
    int num_poses = poses.size();
    Eigen::Matrix4f Tvel2cam = loadVel2Cam("/home/boroson/data/kitti/dataset/sequences/00/calib.txt");

    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud_vel_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud_cam_frame_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    char filename[10];
    sprintf(filename, "%06d.bin", frame_number);
    string scan_file = data_path + string(filename);
    cout << "Read file " << filename << endl;

    read_scan(velo_cloud_vel_frame_ptr, scan_file);

    pcl::transformPointCloud(*velo_cloud_vel_frame_ptr, *velo_cloud_cam_frame_ptr, Tvel2cam);

    cout << "True rotation from this frame to Cam i frame: \n" << poses[frame_number] << endl;

    return velo_cloud_cam_frame_ptr;
}


