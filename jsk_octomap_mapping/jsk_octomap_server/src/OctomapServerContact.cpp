/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <jsk_octomap_server/OctomapServerContact.h>
#include <algorithm>

using namespace octomap;
using octomap_msgs::Octomap;

namespace octomap_server{

OctomapServerContact::OctomapServerContact(ros::NodeHandle private_nh_)
: OctomapServer(private_nh_),
  m_octree_contact(NULL),
  m_offsetVisualizeUnknown(0),
  m_occupancyMinX(-std::numeric_limits<double>::max()),
  m_occupancyMaxX(std::numeric_limits<double>::max()),
  m_occupancyMinY(-std::numeric_limits<double>::max()),
  m_occupancyMaxY(std::numeric_limits<double>::max())
{
  delete m_octree;
  m_octree = NULL;
  m_octree = new OcTreeContact(m_res);
  m_octree_contact = dynamic_cast<OcTreeContact*>(m_octree);
  if (!m_octree_contact){
    ROS_ERROR("Could not convert OcTreeContact from OcTree");
    assert(m_octree_contact);
  }

  m_useHeightMap = false;

  ros::NodeHandle private_nh(private_nh_);

  private_nh.param("offset_vis_unknown", m_offsetVisualizeUnknown,m_offsetVisualizeUnknown);

  private_nh.param("occupancy_min_x", m_occupancyMinX,m_occupancyMinX);
  private_nh.param("occupancy_max_x", m_occupancyMaxX,m_occupancyMaxX);
  private_nh.param("occupancy_min_y", m_occupancyMinY,m_occupancyMinY);
  private_nh.param("occupancy_max_y", m_occupancyMaxY,m_occupancyMaxY);

  double r, g, b, a;
  private_nh.param("color_unknown/r", r, 0.5);
  private_nh.param("color_unknown/g", g, 0.5);
  private_nh.param("color_unknown/b", b, 0.7);
  private_nh.param("color_unknown/a", a, 1.0);
  m_colorUnknown.r = r;
  m_colorUnknown.g = g;
  m_colorUnknown.b = b;
  m_colorUnknown.a = a;

  m_unknownPointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_unknown_point_cloud_centers", 1, m_latchedTopics);
  m_umarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("unknown_cells_vis_array", 1, m_latchedTopics);

  m_contactSensorSub = new message_filters::Subscriber<jsk_recognition_msgs::ContactSensorArray> (m_nh, "contact_sensors_in", 2);
  m_tfContactSensorSub = new tf::MessageFilter<jsk_recognition_msgs::ContactSensorArray> (*m_contactSensorSub, m_tfListener, m_worldFrameId, 2);
  m_tfContactSensorSub->registerCallback(boost::bind(&OctomapServerContact::insertContactSensorCallback, this, _1));

  initContactSensor(private_nh);
}

OctomapServerContact::~OctomapServerContact(){
}

void OctomapServerContact::initContactSensor(ros::NodeHandle private_nh_){
  private_nh_.param<double>("min_sensor_dist", min_sensor_dist_, 0.01);
  double default_padding, default_scale;
  private_nh_.param<double>("self_see_default_padding", default_padding, .01);
  private_nh_.param<double>("self_see_default_scale", default_scale, 1.0);
  std::vector<robot_self_filter::LinkInfo> links;
  std::string link_names;

  if(!private_nh_.hasParam("self_see_links")) {
    ROS_WARN("No links specified for self filtering.");
  } else {
    XmlRpc::XmlRpcValue ssl_vals;;
    private_nh_.getParam("self_see_links", ssl_vals);
    if(ssl_vals.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("Self see links need to be an array");
    } else {
      if(ssl_vals.size() == 0) {
        ROS_WARN("No values in self see links array");
      } else {
        for(int i = 0; i < ssl_vals.size(); i++) {
          robot_self_filter::LinkInfo li;
          if(ssl_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN("Self see links entry %d is not a structure.  Stopping processing of self see links",i);
            break;
          }
          if(!ssl_vals[i].hasMember("name")) {
            ROS_WARN("Self see links entry %d has no name.  Stopping processing of self see links",i);
            break;
          }
          li.name = std::string(ssl_vals[i]["name"]);
          if(!ssl_vals[i].hasMember("padding")) {
            ROS_DEBUG("Self see links entry %d has no padding.  Assuming default padding of %g",i,default_padding);
            li.padding = default_padding;
          } else {
            li.padding = ssl_vals[i]["padding"];
          }
          if(!ssl_vals[i].hasMember("scale")) {
            ROS_DEBUG("Self see links entry %d has no scale.  Assuming default scale of %g",i,default_scale);
            li.scale = default_scale;
          } else {
            li.scale = ssl_vals[i]["scale"];
          }
          links.push_back(li);
        }
      }
    }
  }
  m_selfMask = new robot_self_filter::SelfMask<pcl::PointXYZ>(m_tfListener, links);
}

void OctomapServerContact::insertContactSensor(std::vector<jsk_recognition_msgs::ContactSensor> datas){
  std_msgs::Header tmpHeader;
  tmpHeader.frame_id = m_worldFrameId;
  tmpHeader.stamp = ros::Time::now();

  point3d pmin( m_occupancyMinX, m_occupancyMinY, m_occupancyMinZ);
  point3d pmax( m_occupancyMaxX, m_occupancyMaxY, m_occupancyMaxZ);
  float diff[3];
  unsigned int steps[3];
  double resolution = m_octree_contact->getResolution();
  for (int i=0;i<3;++i) {
    diff[i] = pmax(i) - pmin(i);
    steps[i] = floor(diff[i] / resolution);
    //      std::cout << "bbx " << i << " size: " << diff[i] << " " << steps[i] << " steps\n";
  }

  m_selfMask->assumeFrame(tmpHeader);

  // loop for grids of octomap
  std::vector< std::vector<bool> > contain_flag(datas.size(), std::vector<bool>(8));
  point3d p = pmin;
  for (unsigned int x=0; x<steps[0]; ++x) {
    p.x() += resolution;
    for (unsigned int y=0; y<steps[1]; ++y) {
      p.y() += resolution;
      for (unsigned int z=0; z<steps[2]; ++z) {
        // std::cout << "querying p=" << p << std::endl;
        p.z() += resolution;
        // loop for vertices of each gird
        point3d vertex_offset(-resolution/2.0, -resolution/2.0, -resolution/2.0);
        point3d vertex;
        for (int i=0; i<2; i++) {
          if (i==1) { vertex_offset.z() += resolution; }
          for (int j=0; j<2; j++) {
            if (j==1) { vertex_offset.y() += resolution; }
            for (int k=0; k<2; k++) {
              if(k==1) { vertex_offset.x() += resolution; }
              vertex = p + vertex_offset;
              // std::cout << "vertex = " << vertex << std::endl;
              // loop for each body link
              for (int l=0; l<datas.size(); l++) {
                if (m_selfMask->getMaskContainmentforNamedLink(vertex(0), vertex(1), vertex(2), datas[l].link_name) == robot_self_filter::INSIDE) {
                  // std::cout << "inside vertex = " << vertex << std::endl;
                  contain_flag[l][i+j*2+k*4] = true;
                } else {
                  contain_flag[l][i+j*2+k*4] = false;
                }
              }
            }
            vertex_offset.x() -= resolution;
          }
          vertex_offset.y() -= resolution;
        }

        // update probability of grid
        std::vector<bool> contain_flag_link_sum(8, false);
        std::vector<bool> contain_flag_vertices_sum(datas.size(), false);
        std::vector<bool> contain_flag_vertices_prod(datas.size(), true);
        bool inside_flag = false;
        bool surface_flag = false;
        for (int l=0; l<datas.size(); l++) {
          for (int i=0; i<8; i++) {
            if(contain_flag[l][i]) {
              contain_flag_link_sum[i] = true;
              contain_flag_vertices_sum[l] = true;
            } else {
              contain_flag_vertices_prod[l] = false;
            }
          }
        }
        inside_flag = (std::find(contain_flag_link_sum.begin(), contain_flag_link_sum.end(), false) == contain_flag_link_sum.end()); // when all elements is true
        for (int l=0; l<datas.size(); l++) {
          if(contain_flag_vertices_sum[l] && !(contain_flag_vertices_prod[l]) ) {
            if(datas[l].contact) {
              surface_flag = true;
            }
          }
        }
        if(inside_flag) { // inside
          octomap::OcTreeKey pKey;
          if (m_octree_contact->coordToKeyChecked(p, pKey)) {
            m_octree_contact->updateNode(pKey, m_octree_contact->getProbMissContactSensorLog());
            // std::cout << "find inside grid and find key. p = " << vertex << std::endl;
          }
        }
        else if(surface_flag) { // surface
          octomap::OcTreeKey pKey;
          if (m_octree_contact->coordToKeyChecked(p, pKey)) {
            m_octree_contact->updateNode(pKey, m_octree_contact->getProbHitContactSensorLog());
            // std::cout << "find surface grid and find key. p = " << vertex << std::endl;
          }
        }
      }
      p.z() = pmin.z();
    }
    p.y() = pmin.y();
  }
  m_octree_contact->updateInnerOccupancy();
}

void OctomapServerContact::insertContactSensorCallback(const jsk_recognition_msgs::ContactSensorArray::ConstPtr& msg){
  ROS_WARN_STREAM("insert contact sensor");
  std::vector<jsk_recognition_msgs::ContactSensor> datas = msg->datas;
  insertContactSensor(datas);

  publishAll(msg->header.stamp);
}

void OctomapServerContact::publishAll(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octree_contact->size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
  bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
  m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

  // init markers for free space:
  visualization_msgs::MarkerArray freeNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  freeNodesVis.markers.resize(m_treeDepth+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // init pointcloud:
  pcl::PointCloud<pcl::PointXYZ> pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTree::iterator it = m_octree_contact->begin(m_maxTreeDepth),
      end = m_octree_contact->end(); it != end; ++it)
  {
    bool inUpdateBBX = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (inUpdateBBX)
      handleNodeInBBX(it);

    if (m_octree_contact->isNodeOccupied(*it)){
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      if (x > m_occupancyMinX && x < m_occupancyMaxX && y > m_occupancyMinY && y < m_occupancyMaxY && z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();

        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
          ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (inUpdateBBX)
          handleOccupiedNodeInBBX(it);


        //create marker:
        if (publishMarkerArray){
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          if (m_useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octree_contact->getMetricMin(minX, minY, minZ);
            m_octree_contact->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }
        }

        // insert into pointcloud:
        if (publishPointCloud)
          pclCloud.push_back(pcl::PointXYZ(x, y, z));

      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      if (x > m_occupancyMinX && x < m_occupancyMaxX && y > m_occupancyMinY && y < m_occupancyMaxY && z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        handleFreeNode(it);
        if (inUpdateBBX)
          handleFreeNodeInBBX(it);

        if (m_publishFreeSpace){
          double x = it.getX();
          double y = it.getY();

          //create marker for free space:
          if (publishFreeMarkerArray){
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }

      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octree_contact->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);
  }


  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_octree_contact->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_fmarkerPub.publish(freeNodesVis);
  }

  // publish unknown grid as marker
  visualization_msgs::MarkerArray unknownNodesVis;
  unknownNodesVis.markers.resize(1);

  point3d_list unknown_leaves;
  double offset = m_offsetVisualizeUnknown;
  point3d p_min( m_occupancyMinX + offset, m_occupancyMinY + offset, m_occupancyMinZ + offset);
  point3d p_max( m_occupancyMaxX - offset, m_occupancyMaxY - offset, m_occupancyMaxZ - offset);

  m_octree_contact->getUnknownLeafCenters(unknown_leaves, p_min, p_max);
  pcl::PointCloud<pcl::PointXYZ> unknownCloud;

  for(point3d_list::iterator it = unknown_leaves.begin(); it != unknown_leaves.end(); it++){
    float x = (*it).x();
    float y = (*it).y();
    float z = (*it).z();
    unknownCloud.push_back(pcl::PointXYZ(x, y, z));

    geometry_msgs::Point cubeCenter;
    cubeCenter.x = x;
    cubeCenter.y = y;
    cubeCenter.z = z;
    if (m_useHeightMap){
      double minX, minY, minZ, maxX, maxY, maxZ;
      m_octree_contact->getMetricMin(minX, minY, minZ);
      m_octree_contact->getMetricMax(maxX, maxY, maxZ);
      double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
      unknownNodesVis.markers[0].colors.push_back(heightMapColor(h));
    }
    unknownNodesVis.markers[0].points.push_back(cubeCenter);
  }

  double size = m_octree_contact->getNodeSize(m_maxTreeDepth);
  unknownNodesVis.markers[0].header.frame_id = m_worldFrameId;
  unknownNodesVis.markers[0].header.stamp = rostime;
  unknownNodesVis.markers[0].ns = "map";
  unknownNodesVis.markers[0].id = 0;
  unknownNodesVis.markers[0].type = visualization_msgs::Marker::CUBE_LIST;
  unknownNodesVis.markers[0].scale.x = size;
  unknownNodesVis.markers[0].scale.y = size;
  unknownNodesVis.markers[0].scale.z = size;
  unknownNodesVis.markers[0].color = m_colorUnknown;

  if (unknownNodesVis.markers[0].points.size() > 0)
    unknownNodesVis.markers[0].action = visualization_msgs::Marker::ADD;
  else
    unknownNodesVis.markers[0].action = visualization_msgs::Marker::DELETE;
  m_umarkerPub.publish(unknownNodesVis);

  // publish unknown grid as pointcloud
  sensor_msgs::PointCloud2 unknown_ros_cloud;
  pcl::toROSMsg (unknownCloud, unknown_ros_cloud);
  unknown_ros_cloud.header.frame_id = m_worldFrameId;
  unknown_ros_cloud.header.stamp = rostime;
  m_unknownPointCloudPub.publish(unknown_ros_cloud);

  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  if (publishBinaryMap)
    publishBinaryOctoMap(rostime);

  if (publishFullMap)
    publishFullOctoMap(rostime);


  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

}

}



