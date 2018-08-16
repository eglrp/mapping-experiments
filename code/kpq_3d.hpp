/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef KPQ_KEYPOINT3D_IMPL_H_
#define KPQ_KEYPOINT3D_IMPL_H_

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>

#include "kpq_3d.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setSalientRadius (double salient_radius)
{
  salient_radius_ = salient_radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setNonMaxRadius (double non_max_radius)
{
  non_max_radius_ = non_max_radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setNormalRadius (double normal_radius)
{
  normal_radius_ = normal_radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setBorderRadius (double border_radius)
{
  border_radius_ = border_radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setDeltaThreshold (double t1)
{
  t1_ = t1;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setDeltaMax (double t2)
{
    t2_ = t2;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setMinNeighbors (int min_neighbors)
{
  min_neighbors_ = min_neighbors;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::setNormals (const PointCloudNConstPtr &normals)
{
  normals_ = normals;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> bool*
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::getBoundaryPoints (PointCloudIn &input, double border_radius, float angle_threshold)
{
  bool* edge_points = new bool [input.size ()];

  Eigen::Vector4f u = Eigen::Vector4f::Zero ();
  Eigen::Vector4f v = Eigen::Vector4f::Zero ();

  pcl::BoundaryEstimation<PointInT, NormalT, pcl::Boundary> boundary_estimator;
  boundary_estimator.setInputCloud (input_);

  int index;
#ifdef _OPENMP
#pragma omp parallel for private(u, v) num_threads(threads_)
#endif
  for (index = 0; index < int (input.points.size ()); index++)
  {
    edge_points[index] = false;
    PointInT current_point = input.points[index];

    if (pcl::isFinite(current_point))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_distances;
      int n_neighbors;

      this->searchForNeighbors (static_cast<int> (index), border_radius, nn_indices, nn_distances);

      n_neighbors = static_cast<int> (nn_indices.size ());

      if (n_neighbors >= min_neighbors_)
      {
	boundary_estimator.getCoordinateSystemOnPlane (normals_->points[index], u, v);

	if (boundary_estimator.isBoundaryPoint (input, static_cast<int> (index), nn_indices, u, v, angle_threshold))
	  edge_points[index] = true;
      }
    }
  }

  return (edge_points);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> float
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::computeDelta (const int& current_index)
{
  const PointInT& current_point = (*input_).points[current_index];
  const NormalT& current_normal = (*normals_).points[current_index];

  float central_point[3];
  memset(central_point, 0, sizeof(double) * 3);

//  central_point[0] = current_point.x;
//  central_point[1] = current_point.y;
//  central_point[2] = current_point.z;

  Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero ();

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  int n_neighbors;

  this->searchForNeighbors (current_index, salient_radius_, nn_indices, nn_distances);

  n_neighbors = static_cast<int> (nn_indices.size ());

  if (n_neighbors < min_neighbors_)
    return 0;

    double cov[9];
  memset(cov, 0, sizeof(double) * 9);

  Eigen::Matrix<double , 3, Eigen::Dynamic> Lprime1 = Eigen::MatrixXd::Zero(3, n_neighbors);
  Eigen::Matrix<double , 3, Eigen::Dynamic> Lprime = Eigen::MatrixXd::Zero(3, n_neighbors);

    // Rotate all points to frame with normal in direction [0,0,1] - find rotation to make
  // Step 1: Find axis (X)
  Eigen::Vector3f normal_vec(current_normal.normal_x, current_normal.normal_y, current_normal.normal_z);
  Eigen::Vector3f normal_goal_vec(0, 0, 1);
  Eigen::Vector3f crossProduct = normal_vec.cross(normal_goal_vec);
  float crossProductNorm = crossProduct.norm();
  Eigen::Vector3f vector_X = (crossProduct / crossProductNorm);

// Step 2: Find angle (theta)
  float dotProduct = normal_vec.dot(normal_goal_vec);
  float norm_A = normal_vec.norm();
  float norm_B = normal_goal_vec.norm();
  float dotProductOfNorms = norm_A * norm_B;
  float dotProductDividedByDotProductOfNorms = (dotProduct / dotProductOfNorms);
  float thetaAngleRad = acosf(dotProductDividedByDotProductOfNorms);

// Step 3: Construct A, the skew-symmetric matrix corresponding to X
  Eigen::Matrix3f matrix_A = Eigen::Matrix3f::Identity();

  matrix_A(0,0) = 0.0;
  matrix_A(0,1) = -1.0 * (vector_X(2));
  matrix_A(0,2) = vector_X(1);
  matrix_A(1,0) = vector_X(2);
  matrix_A(1,1) = 0.0;
  matrix_A(1,2) = -1.0 * (vector_X(0));
  matrix_A(2,0) = -1.0 * (vector_X(1));
  matrix_A(2,1) = vector_X(0);
  matrix_A(2,2) = 0.0;

// Step 4: Plug and chug.
  Eigen::Matrix3f IdentityMat = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f firstTerm = sinf(thetaAngleRad) * matrix_A;
  Eigen::Matrix3f secondTerm = (1.0 - cosf(thetaAngleRad)) * matrix_A * matrix_A;
  Eigen::Matrix3f transform = IdentityMat + firstTerm + secondTerm;

  // covariance matrix subtracts out mean instead of center point
  for (int n_idx = 0; n_idx < n_neighbors; n_idx++)
  {
    const PointInT& n_point = (*input_).points[nn_indices[n_idx]];
    Eigen::Vector3f n_point_cframe(n_point.x - current_point.x, n_point.y - current_point.y, n_point.z - current_point.z);
    Eigen::Vector3f n_point_cframe_rot = transform * n_point_cframe;
    central_point[0] += n_point_cframe_rot(0);
    central_point[1] += n_point_cframe_rot(1);
    central_point[2] += n_point_cframe_rot(2);
  }

  central_point[0] /= n_neighbors;
  central_point[1] /= n_neighbors;
  central_point[2] /= n_neighbors;

  for (int n_idx = 0; n_idx < n_neighbors; n_idx++)
  {
    const PointInT& n_point = (*input_).points[nn_indices[n_idx]];
      Eigen::Vector3f n_point_cframe(n_point.x - current_point.x, n_point.y - current_point.y, n_point.z - current_point.z);
      Eigen::Vector3f n_point_cframe_rot = transform * n_point_cframe;

    float neigh_point[3];
    memset(neigh_point, 0, sizeof(float) * 3);

    neigh_point[0] = n_point_cframe_rot(0);
    neigh_point[1] = n_point_cframe_rot(1);
    neigh_point[2] = n_point_cframe_rot(2);

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        cov[i * 3 + j] += (1.0/n_neighbors)* ((double)neigh_point[i] - (double)central_point[i]) * ((double)neigh_point[j] - (double)central_point[j]);

    for (int i = 0; i < 3; i++)
      Lprime1(i, n_idx) = (double)neigh_point[i] - (double)central_point[i];
  }

  cov_m << cov[0], cov[1], cov[2],
	   cov[3], cov[4], cov[5],
	   cov[6], cov[7], cov[8];


  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver (cov_m);
  Lprime = solver.eigenvectors() * Lprime1;

  float delta = float((Lprime.row(0).maxCoeff() - Lprime.row(0).minCoeff()) / (Lprime.row(1).maxCoeff() - Lprime.row(1).minCoeff()));

  return delta;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> bool
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::initCompute ()
{
  if (!Keypoint<PointInT, PointOutT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] init failed!\n", name_.c_str ());
    return (false);
  }
  if (salient_radius_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the salient radius (%f) must be strict positive!\n",
		name_.c_str (), salient_radius_);
    return (false);
  }
  if (non_max_radius_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the non maxima radius (%f) must be strict positive!\n",
		name_.c_str (), non_max_radius_);
    return (false);
  }
  if (t1_ <= 1)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the threshold on delta must be grater than 1!\n",
		name_.c_str (), t1_);
    return (false);
  }
  if (min_neighbors_ <= 0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the minimum number of neighbors (%f) must be strict positive!\n",
		name_.c_str (), min_neighbors_);
    return (false);
  }

  if (deltas_)
    delete[] deltas_;

  deltas_ = new double[input_->size ()];
  memset(deltas_, 0, sizeof(double) * input_->size ());

  if (edge_points_)
    delete[] edge_points_;

  if (border_radius_ > 0.0)
  {
    if (normals_->empty ())
    {
      if (normal_radius_ <= 0.)
      {
        PCL_ERROR ("[pcl::%s::initCompute] : the radius used to estimate surface normals (%f) must be positive!\n",
        name_.c_str (), normal_radius_);
        return (false);
      }

      PointCloudNPtr normal_ptr (new PointCloudN ());
      if (input_->height == 1 )
      {
        pcl::NormalEstimation<PointInT, NormalT> normal_estimation;
        normal_estimation.setInputCloud (surface_);
        normal_estimation.setRadiusSearch (normal_radius_);
        normal_estimation.compute (*normal_ptr);
      }
      else
      {
        pcl::IntegralImageNormalEstimation<PointInT, NormalT> normal_estimation;
        normal_estimation.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<PointInT, NormalT>::SIMPLE_3D_GRADIENT);
        normal_estimation.setInputCloud (surface_);
        normal_estimation.setNormalSmoothingSize (5.0);
        normal_estimation.compute (*normal_ptr);
      }
      normals_ = normal_ptr;
    }
    if (normals_->size () != surface_->size ())
    {
      PCL_ERROR ("[pcl::%s::initCompute] normals given, but the number of normals does not match the number of input points!\n", name_.c_str ());
      return (false);
    }
  }
  else if (border_radius_ < 0.0)
  {
    PCL_ERROR ("[pcl::%s::initCompute] : the border radius used to estimate boundary points (%f) must be positive!\n",
		name_.c_str (), border_radius_);
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointInT, typename PointOutT, typename NormalT> void
pcl::KPQKeypoint3D<PointInT, PointOutT, NormalT>::detectKeypoints (PointCloudOut &output)
{
  // Make sure the output cloud is empty
  output.points.clear ();

  if (border_radius_ > 0.0)
    edge_points_ = getBoundaryPoints (*(input_->makeShared ()), border_radius_, angle_threshold_);

  bool* borders = new bool [input_->size()];

  int index;
#ifdef _OPENMP
  #pragma omp parallel for num_threads(threads_)
#endif
  for (index = 0; index < int (input_->size ()); index++)
  {
    borders[index] = false;
    PointInT current_point = input_->points[index];

    if ((border_radius_ > 0.0) && (pcl::isFinite(current_point)))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_distances;

      this->searchForNeighbors (static_cast<int> (index), border_radius_, nn_indices, nn_distances);

      for (size_t j = 0 ; j < nn_indices.size (); j++)
      {
        if (edge_points_[nn_indices[j]])
        {
          borders[index] = true;
          break;
        }
      }
    }
  }

#ifdef _OPENMP
  Eigen::Vector3d *omp_mem = new Eigen::Vector3d[threads_];

  for (size_t i = 0; i < threads_; i++)
    omp_mem[i].setZero (3);
#else
  Eigen::Vector3d *omp_mem = new Eigen::Vector3d[1];
  float delta = 0;

  omp_mem[0].setZero (3);
#endif

  double *prg_local_mem = new double[input_->size () * 3];
  double **prg_mem = new double * [input_->size ()];

  for (size_t i = 0; i < input_->size (); i++)
    prg_mem[i] = prg_local_mem + 3 * i;

#ifdef _OPENMP
  #pragma omp parallel for num_threads(threads_)
#endif
  for (index = 0; index < static_cast<int> (input_->size ()); index++)
  {
#ifdef _OPENMP
    int tid = omp_get_thread_num ();
#else
    int tid = 0;
#endif
    PointInT current_point = input_->points[index];

    if ((!borders[index]) && pcl::isFinite(current_point))
    {
      //if the considered point is not a border point and the point is "finite", then compute the scatter (covariance) matrix
      Eigen::Matrix3d cov_m = Eigen::Matrix3d::Zero ();
      delta = computeDelta (static_cast<int> (index));
//
//      if (!pcl_isfinite (e1c) || !pcl_isfinite (e2c) || !pcl_isfinite (e3c))
//	continue;
//
//      if (e3c < 0)
//      {
//	PCL_WARN ("[pcl::%s::detectKeypoints] : The third eigenvalue is negative! Skipping the point with index %i.\n",
//	          name_.c_str (), index);
//	continue;
//      }
//
//      omp_mem[tid][0] = e2c / e1c;
//      omp_mem[tid][1] = e3c / e2c;;
//      omp_mem[tid][2] = e3c;
    }

    prg_mem[index][0] = delta;
//    for (int d = 0; d < omp_mem[tid].size (); d++)
//        prg_mem[index][d] = omp_mem[tid][d];
  }

  for (index = 0; index < int (input_->size ()); index++)
  {
   if (!borders[index])
    {
        deltas_[index] = prg_mem[index][0];
    }
  }

  bool* feat_max = new bool [input_->size()];
  bool is_max;

#ifdef _OPENMP
  #pragma omp parallel for private(is_max) num_threads(threads_)
#endif
  for (index = 0; index < int (input_->size ()); index++)
  {
    feat_max [index] = false;
    PointInT current_point = input_->points[index];

    if ((deltas_[index] > t1_) && (deltas_[index] < t2_) && (pcl::isFinite(current_point)))
    {
      std::vector<int> nn_indices;
      std::vector<float> nn_distances;
      int n_neighbors;

      this->searchForNeighbors (static_cast<int> (index), non_max_radius_, nn_indices, nn_distances);

      n_neighbors = static_cast<int> (nn_indices.size ());

      if (n_neighbors >= min_neighbors_)
      {
        is_max = true;

        for (int j = 0 ; j < n_neighbors; j++)
          if (deltas_[index] < deltas_[nn_indices[j]])
            is_max = false;
        if (is_max)
          feat_max[index] = true;
      }
    }
  }

#ifdef _OPENMP
#pragma omp parallel for shared (output) num_threads(threads_)
#endif
  for (index = 0; index < int (input_->size ()); index++)
  {
    if (feat_max[index])
#ifdef _OPENMP
#pragma omp critical
#endif
    {
      PointOutT p;
      p.getVector3fMap () = input_->points[index].getVector3fMap ();
      output.points.push_back(p);
      keypoints_indices_->indices.push_back (index);
    }
  }

  output.header = input_->header;
  output.width = static_cast<uint32_t> (output.points.size ());
  output.height = 1;

  // Clear the contents of variables and arrays before the beginning of the next computation.
  if (border_radius_ > 0.0)
    normals_.reset (new pcl::PointCloud<NormalT>);

  delete[] borders;
  delete[] prg_mem;
  delete[] prg_local_mem;
  delete[] feat_max;
  delete[] omp_mem;
}

#define PCL_INSTANTIATE_KPQKeypoint3D(T,U,N) template class PCL_EXPORTS pcl::KPQKeypoint3D<T,U,N>;

#endif /* KPQ_3D_IMPL_H_ */
