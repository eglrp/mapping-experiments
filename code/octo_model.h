//
// Created by boroson on 3/27/18.
//

#ifndef CODE_OCTO_MODEL_H
#define CODE_OCTO_MODEL_H

#include <octomap/octomap.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace octomap;

class octo_model {

public:
    octo_model();
    ~octo_model();

    /*!
     * \brief Constructor
     *
     * Constructs a 3D model from given octomap
     */
    octo_model(OcTree* map);

    cv::Mat gen_depth_image(pose6d cam_pose, cv::Mat camMat, int imsizex, int imsizey);

protected:
    OcTree *map;


};


#endif //CODE_OCTO_MODEL_H
