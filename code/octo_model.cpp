//
// Created by boroson on 3/27/18.
//

#include "octo_model.h"


octo_model::octo_model(){
    map = new OcTree(0.1); //default resolution
}

octo_model::~octo_model() {
    map->clear();
    delete map;
}

octo_model::octo_model(OcTree* area_map) {
    map = area_map;
}

cv::Mat octo_model::gen_depth_image(pose6d cam_pose, cv::Mat camMat, int imsizex, int imsizey) {
    point3d direction;
    point3d ray_end;

    point3d cam_origin = cam_pose.trans();

    cv::Mat depth_img = cv::Mat::zeros(imsizey, imsizex, CV_8UC1);

//    uint8_t img[200][200]

    for (int yi = 0; yi < imsizey; ++yi) {
        for (int xi = 0; xi < imsizex; ++xi) {
//  for(float z = 0; z <= 0.25; z += 0.125){
            direction = point3d(1, -(xi - camMat.at<float>(0,2))/camMat.at<float>(0,0), -(yi - camMat.at<float>(1,2))/camMat.at<float>(1,1));
            direction.rotate_IP(cam_pose.roll(), cam_pose.pitch(), cam_pose.yaw());
//  cout << endl;
//            cout << "casting ray from " << origin  << " in the " << direction << " direction"<< endl;
            bool success = map->castRay(cam_origin, direction, ray_end);

            if (success) {
                depth_img.at<uchar>(yi,xi) = uint8_t(ray_end.x()*10);
//        cout << "entrance point is " << intersection << endl;
            }
        }
    }
    return depth_img;

}