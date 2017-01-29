// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
 *********************************************************************/

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/edge_based_pose_estimation.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


namespace jsk_pcl_ros
{
  void EdgeBasedPoseEstimation::onInit()
  {
    ConnectionBasedNodelet::onInit();

    pnh_->param("max_queue_size", max_queue_size_, 10);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output_pose", 1);
    onInitPostProcess();
  }

  void EdgeBasedPoseEstimation::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sub_edges_.subscribe(*pnh_, "edges", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(max_queue_size_);
      async_->connectInput(sub_indices_, sub_cloud_, sub_edges_);
      async_->registerCallback(boost::bind(&EdgeBasedPoseEstimation::estimate, this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(max_queue_size_);
      sync_->connectInput(sub_indices_, sub_cloud_, sub_edges_);
      sync_->registerCallback(boost::bind(&EdgeBasedPoseEstimation::estimate, this, _1, _2, _3));
    }
  }

  void EdgeBasedPoseEstimation::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_indices_.unsubscribe();
    sub_edges_.unsubscribe();
  }

  void EdgeBasedPoseEstimation::estimate(
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::EdgeArray::ConstPtr& edges_msg)
  {
    geometry_msgs::PoseStamped out_pose_msg;
    pub_.publish(out_pose_msg);

    ROS_INFO("[edge_based_pose_estimation_nodelet] esitmate is called.\n");
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EdgeBasedPoseEstimation, nodelet::Nodelet);
