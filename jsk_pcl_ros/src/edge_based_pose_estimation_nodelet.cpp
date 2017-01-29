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

#include <pcl/filters/extract_indices.h>


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
    NODELET_INFO("estimate function is called.\n");

    int indices_size = indices_msg->cluster_indices.size();
    int edges_size = edges_msg->edges.size();
    if(indices_size != edges_size) {
      NODELET_ERROR("size of indices(%d) and edges(%d) are not same.\n", indices_size, edges_size);
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr detected_edge_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointNormal>::Ptr model_edge_cloud(new pcl::PointCloud<pcl::PointNormal>());

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::vector<pcl::IndicesPtr> indices_vec;

    // get indices vec
    for (size_t i = 0; i < indices_msg->cluster_indices.size(); i++) {
      pcl::IndicesPtr indices;
      indices.reset (new std::vector<int> (indices_msg->cluster_indices[i].indices));
      indices_vec.push_back(indices);
    }

    // get pointcloud on detected edges and pointcloud of model edges
    pcl::fromROSMsg(*cloud_msg, *cloud);
    extract.setInputCloud(cloud);
    for (size_t i = 0; i < indices_vec.size(); i++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      extract.setIndices(indices_vec[i]);
      extract.filter(*segmented_cloud);

      // merge detected edge cloud
      *detected_edge_cloud += *segmented_cloud;
      detected_edge_cloud->width = (int)detected_edge_cloud->points.size();
      detected_edge_cloud->height = 1;

      // generate point for model edge cloud
      Eigen::Vector3d start_point;
      Eigen::Vector3d end_point;
      Eigen::Vector3d edge_dir;
      tf::pointMsgToEigen(edges_msg->edges[i].start_point, start_point);
      tf::pointMsgToEigen(edges_msg->edges[i].end_point, end_point);
      edge_dir = end_point - start_point;
      pcl::PointNormal model_edge_point;
      model_edge_point.x = start_point(0);
      model_edge_point.y = start_point(1);
      model_edge_point.z = start_point(2);
      model_edge_point.normal_x = edge_dir(0);
      model_edge_point.normal_y = edge_dir(1);
      model_edge_point.normal_z = edge_dir(2);

      // merge model edge cloud
      for (size_t j = 0; j < indices_vec[i]->size(); j++) {
        model_edge_cloud->points.push_back(model_edge_point);
      }
      model_edge_cloud->width = (int)model_edge_cloud->points.size();
      model_edge_cloud->height = 1;
    }

    // estimate transformation
    Eigen::Affine3f trans;
    trans_est_.estimateRigidTransformation(*detected_edge_cloud, *model_edge_cloud, trans.matrix());
    trans = trans.inverse(); // inverse transformation because src and dest is flipped in estimation

    std::cout << "estimated transformation" << std::endl << trans.matrix() << std::endl;

    geometry_msgs::PoseStamped out_pose_msg;
    pub_.publish(out_pose_msg);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EdgeBasedPoseEstimation, nodelet::Nodelet);
