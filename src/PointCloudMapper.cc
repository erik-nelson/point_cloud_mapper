/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <point_cloud_mapper/PointCloudMapper.h>
#include <parameter_utils/ParameterUtils.h>

namespace pu = parameter_utils;

PointCloudMapper::PointCloudMapper() : initialized_(false) {
  // Initialize map data container.
  map_data_.reset(new PointCloud);
}

PointCloudMapper::~PointCloudMapper() {}

bool PointCloudMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudMapper");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudMapper::LoadParameters(const ros::NodeHandle& n) {
  // Load fixed frame.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;

  // Load map parameters.
  if (!pu::Get("map/octree_resolution", octree_resolution_)) return false;

  // Initialize the map octree.
  map_octree_.reset(new Octree(octree_resolution_));
  map_octree_->setInputCloud(map_data_);

  initialized_ = true;

  return true;
}

bool PointCloudMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  map_pub_ = nl.advertise<PointCloud>("octree_map", 10, false);

  return true;
}

bool PointCloudMapper::InsertPoints(const PointCloud::ConstPtr& points,
                                    bool populate_incremental_points,
                                    PointCloud* incremental_points) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
    return false;
  }

  if (populate_incremental_points && incremental_points == NULL) {
    ROS_ERROR("%s: Incremental point cloud argument is null.", name_.c_str());
    return false;
  } else if (populate_incremental_points) {
    incremental_points->clear();
  }

  // Iterate over points in the input point cloud, inserting them into the map
  // if there is not already a point in the same voxel.
  for (size_t ii = 0; ii < points->points.size(); ++ii) {
    const pcl::PointXYZ p = points->points[ii];
    if (!map_octree_->isVoxelOccupiedAtPoint(p)) {
      map_octree_->addPointToCloud(p, map_data_);

      // Add the new point to an incremental point cloud for visualization.
      if (populate_incremental_points) {
        incremental_points->push_back(p);
      }
    }
  }

  return true;
}

bool PointCloudMapper::ApproxNearestNeighbors(const PointCloud& points,
                                              PointCloud* neighbors) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
  }

  if (neighbors == NULL) {
    ROS_ERROR("%s: Output argument is null.", name_.c_str());
  }

  neighbors->points.clear();

  // Iterate over points in the input point cloud, finding the nearest neighbor
  // for every point and storing it in the output array.
  for (size_t ii = 0; ii < points.points.size(); ++ii) {
    // Search for nearest neighbor and store.
    float unused = 0.f;
    int result_index = -1;

    map_octree_->approxNearestSearch(points.points[ii], result_index, unused);
    if (result_index >= 0)
      neighbors->push_back(map_data_->points[result_index]);
  }

  return neighbors->points.size() > 0;
}

void PointCloudMapper::PublishMap() const {
  if (initialized_ && map_pub_.getNumSubscribers() > 0) {
    map_data_->header.frame_id = fixed_frame_id_;
    map_pub_.publish(map_data_);
  }
}
