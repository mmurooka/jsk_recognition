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

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_CONTACT_H
#define OCTOMAP_SERVER_OCTOMAPSERVER_CONTACT_H

#include <octomap_server/OctomapServer.h>
#include <jsk_octomap_server/OcTreeContact.h>
#include <jsk_recognition_msgs/ContactSensorArray.h>
#include <pr2_navigation_self_filter/self_mask.h>
//#include <jsk_octomap_server/self_mask_ex.h>


namespace octomap_server {
  enum
  {
    FEASIBLE_POSE = 0,
    SAFE_POSE = 1,
    SAFE_POSE2 = 2,
    UNSAFE_POSE = 3,
    INFEASIBLE_POSE = 4,
  };

class OctomapServerContact : public OctomapServer{

public:

  OctomapServerContact(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
  virtual ~OctomapServerContact();

  void initContactSensor(ros::NodeHandle private_nh_);
  void insertContactSensor(std::vector<jsk_recognition_msgs::ContactSensor> datas);
  virtual void insertContactSensorCallback(const jsk_recognition_msgs::ContactSensorArray::ConstPtr& msg);

  void publishAll(const ros::Time& rostime);

  ros::Publisher m_unknownPointCloudPub, m_umarkerPub;
  message_filters::Subscriber<jsk_recognition_msgs::ContactSensorArray>* m_contactSensorSub;
  tf::MessageFilter<jsk_recognition_msgs::ContactSensorArray>* m_tfContactSensorSub;
  ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_clearBBXService, m_resetService;

  std_msgs::ColorRGBA m_colorUnknown;

  double m_offsetVisualizeUnknown;

  double m_occupancyMinX;
  double m_occupancyMaxX;
  double m_occupancyMinY;
  double m_occupancyMaxY;

  robot_self_filter::SelfMask<pcl::PointXYZ>* m_selfMask;
  double min_sensor_dist_;

  octomap::OcTreeContact* m_octree_contact;
};
}

#endif
