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
#include <boost/thread/thread.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_broadcaster.h>

#include "jsk_pcl_ros/edge_based_pose_estimation.h"


namespace jsk_pcl_ros
{
  void EdgeBasedPoseEstimation::onInit()
  {
    ConnectionBasedNodelet::onInit();

    pnh_->param("max_queue_size", max_queue_size_, 10);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("debug_viewer", debug_viewer_, false);
    pnh_->param("publish_tf", publish_tf_, false);
    pnh_->param("output_frame_id", output_frame_id_, std::string("track_result"));
    pub_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output_pose", 1);
    onInitPostProcess();
  }

  void EdgeBasedPoseEstimation::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sub_edges_.subscribe(*pnh_, "edges", 1);
    sub_pose_.subscribe(*pnh_, "pose", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(max_queue_size_);
      async_->connectInput(sub_indices_, sub_cloud_, sub_edges_, sub_pose_);
      async_->registerCallback(boost::bind(&EdgeBasedPoseEstimation::estimate, this, _1, _2, _3, _4));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(max_queue_size_);
      sync_->connectInput(sub_indices_, sub_cloud_, sub_edges_, sub_pose_);
      sync_->registerCallback(boost::bind(&EdgeBasedPoseEstimation::estimate, this, _1, _2, _3, _4));
    }
  }

  void EdgeBasedPoseEstimation::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_indices_.unsubscribe();
    sub_edges_.unsubscribe();
    sub_pose_.unsubscribe();
  }

  void EdgeBasedPoseEstimation::estimate(
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::EdgeArray::ConstPtr& edges_msg,
    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    NODELET_INFO("estimate function is called.\n");

    // check size
    int indices_size = indices_msg->cluster_indices.size();
    int edges_size = edges_msg->edges.size();
    if(indices_size != edges_size) {
      NODELET_ERROR("size of indices(%d) and edges(%d) are not same.\n", indices_size, edges_size);
      return;
    }

    // check frame_id
    std::string cloud_frame_id = cloud_msg->header.frame_id;
    std::string edges_frame_id = edges_msg->header.frame_id;
    std::string pose_frame_id = pose_msg->header.frame_id;
    if (! ((cloud_frame_id == edges_frame_id) && (cloud_frame_id == pose_frame_id))) {
      NODELET_ERROR("frame_id of indices(%s), edges(%s), and pose(%s) are not same.\n",
                    cloud_frame_id.c_str(), edges_frame_id.c_str(), pose_frame_id.c_str());
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
    Eigen::Affine3f trans_inv;
    trans_est_.estimateRigidTransformation(*detected_edge_cloud, *model_edge_cloud, trans.matrix());
    trans_inv = trans.inverse(); // inverse transformation because src and dest is flipped in estimation

    // std::cout << "estimated transformation" << std::endl << trans.matrix() << std::endl;

    // consider initial transformation
    Eigen::Affine3f initial_trans;
    Eigen::Affine3f out_trans;
    geometry_msgs::PoseStamped out_pose_msg;
    tf::poseMsgToEigen(pose_msg->pose, initial_trans);
    out_trans = trans_inv * initial_trans;
    tf::poseEigenToMsg(out_trans, out_pose_msg.pose);

    // publish out pose
    out_pose_msg.header = cloud_msg->header;
    pub_.publish(out_pose_msg);

    // publish out result
    if (!publish_tf_) {
      tf::Transform tf_out_trans;
      tf::transformEigenToTF(out_trans, tf_out_trans);
      static tf::TransformBroadcaster tf_broadcaster;
      tf_broadcaster.sendTransform(tf::StampedTransform(tf_out_trans, cloud_msg->header.stamp,
                                                        cloud_msg->header.frame_id, output_frame_id_));
    }

    // display debug viewer
    if (debug_viewer_) {
      // transform pointcloud with estimated transformation
      pcl::PointCloud<pcl::PointNormal>::Ptr model_edge_cloud_transformed(new pcl::PointCloud<pcl::PointNormal>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr detected_edge_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*model_edge_cloud, *model_edge_cloud_transformed, trans_inv);
      pcl::transformPointCloud(*detected_edge_cloud, *detected_edge_cloud_transformed, trans);

      // visualize pointcloud
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor(0, 0, 0);
      // model_edge_cloud
      viewer->addPointCloudNormals<pcl::PointNormal>(model_edge_cloud, 6, 1.0, "model_edge_cloud");
      // model_edge_cloud_transformed
      viewer->addPointCloudNormals<pcl::PointNormal>(model_edge_cloud_transformed, 6, 1.0, "model_edge_cloud_transformed");
      // detected_edge_cloud
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_detected_edge_cloud(detected_edge_cloud, 0, 255, 0);
      viewer->addPointCloud<pcl::PointXYZ>(detected_edge_cloud, rgb_detected_edge_cloud, "detected_edge_cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "detected_edge_cloud");
      // detected_edge_cloud_transformed
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb_detected_edge_cloud_transformed(detected_edge_cloud_transformed, 0, 0, 255);
      viewer->addPointCloud<pcl::PointXYZ>(detected_edge_cloud_transformed, rgb_detected_edge_cloud_transformed, "detected_edge_cloud_transformed");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "detected_edge_cloud_transformed");

      while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
      viewer->removeAllPointClouds();
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EdgeBasedPoseEstimation, nodelet::Nodelet);
