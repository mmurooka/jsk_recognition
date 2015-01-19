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

#include "jsk_perception/color_histogram_label_match.h"
#include <cv_bridge/cv_bridge.h>
#include <jsk_topic_tools/color_utils.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void ColorHistogramLabelMatch::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_debug_ = advertise<sensor_msgs::Image>(
      *pnh_, "debug", 1);
  }

  void ColorHistogramLabelMatch::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_label_.subscribe(*pnh_, "input/label", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_image_, sub_label_);
    sync_->registerCallback(
      boost::bind(
        &ColorHistogramLabelMatch::match, this, _1, _2));
    sub_histogram_ = pnh_->subscribe(
      "input/histogram", 1, &ColorHistogramLabelMatch::histogramCallback, this);
  }

  void ColorHistogramLabelMatch::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_label_.unsubscribe();
    sub_histogram_.shutdown();
  }

  void ColorHistogramLabelMatch::getLabels(const cv::Mat& label,
                                           std::vector<int>& keys)
  {
    std::map<int, bool> map;
    for (int j = 0; j < label.rows; j++) {
      for (int i = 0; i < label.cols; i++) {
        int label_value = label.at<int>(j, i);
        if (map.find(label_value) == map.end()) {
          map[label_value] = true;
        }
      }
    }

    for (std::map<int, bool>::iterator it = map.begin();
         it != map.end();
         ++it) {
      keys.push_back(it->first);
    }
  }

  void ColorHistogramLabelMatch::getMaskImage(const cv::Mat& label_image,
                                              const int label,
                                              cv::Mat& mask)
  {
    for (int j = 0; j < label_image.rows; j++) {
      for (int i = 0; i < label_image.cols; i++) {
        if (label_image.at<int>(j, i) == label) {
          mask.at<uchar>(j, i) = 255;
        }
      }
    }
  }
  
  void ColorHistogramLabelMatch::match(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& label_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (histogram_.empty()) {
      NODELET_DEBUG("no reference histogram is available");
      return;
    }
    
    cv_bridge::CvImageConstPtr image_bridge
      = cv_bridge::toCvShare(image_msg, image_msg->encoding);
    cv_bridge::CvImageConstPtr label_bridge
      = cv_bridge::toCvShare(label_msg, label_msg->encoding);
    cv::Mat image = image_bridge->image.clone();
    cv::Mat label = label_bridge->image.clone();
    
    std::vector<int> labels;
    getLabels(label, labels);
    
    cv::Mat coefficients_image = cv::Mat::zeros(image_msg->height,
                                                image_msg->width,
                                                CV_8UC3); // BGR8
    int hist_size = histogram_.cols;
    float range[] = { 0, 256 } ;
    const float* hist_range = { range };
    for (size_t i = 0; i < labels.size(); i++) {
      int label_index = labels[i];
      cv::Mat mask = cv::Mat::zeros(label.rows, label.cols, CV_8UC1);
      getMaskImage(label, label_index, mask);
      cv::MatND hist;
      // NB: normalize...?
      bool uniform = true; bool accumulate = false;
      cv::calcHist(&image, 1, 0, mask, hist, 1,
                   &hist_size, &hist_range, uniform, accumulate);
      cv::Mat hist_mat = cv::Mat::zeros(1, hist_size, CV_32FC1);
      for (size_t j = 0; j < hist_size; j++) {
        hist_mat.at<float>(0, j) = hist.at<float>(0, j);
      }
      
      normalizeHistogram(hist_mat);
      double coef = coefficients(histogram_, hist_mat);
      std_msgs::ColorRGBA coef_color = jsk_topic_tools::heatColor(coef);
      for (size_t j = 0; j < coefficients_image.rows; j++) {
        for (size_t i = 0; i < coefficients_image.cols; i++) {
          if (mask.at<uchar>(j, i) == 255) {
            coefficients_image.at<cv::Vec3b>(j, i)
              = cv::Vec3b(int(coef_color.b * 255),
                          int(coef_color.g * 255),
                          int(coef_color.r * 255));
          }
        }
      }
    }
    pub_debug_.publish(
      cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::BGR8,
                         coefficients_image).toImageMsg());
  }

  void ColorHistogramLabelMatch::normalizeHistogram(
    cv::Mat& histogram)
  {
    float sum = 0.0;
    for (size_t i = 0; i < histogram.cols; i++) {
      sum += histogram.at<float>(0, i);
    }
    for (size_t i = 0; i < histogram.cols; i++) {
      histogram.at<float>(0, i) = histogram.at<float>(0, i) / sum;
    }
  }
  
  double ColorHistogramLabelMatch::coefficients(
    const cv::Mat& ref_hist,
    const cv::Mat& target_hist)
  {
    double sum = 0.0;
    if (ref_hist.cols != target_hist.cols ||
        ref_hist.rows != target_hist.rows) {
      NODELET_FATAL("histogram size is not same");
      NODELET_FATAL("ref_hist: %dx%d", ref_hist.cols, ref_hist.rows);
      NODELET_FATAL("target_hist: %dx%d", target_hist.cols, target_hist.rows);
      return 0;
    }
    // bhattacharyya
    for (size_t i = 0; i < ref_hist.cols; i++) {
      sum += sqrt(ref_hist.at<float>(0, i) * target_hist.at<float>(0, i));
    }
    return sum;
  }

  void ColorHistogramLabelMatch::histogramCallback(
    const jsk_pcl_ros::ColorHistogram::ConstPtr& histogram_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    //histogram_ = histogram_msg->histogram;
    histogram_ = cv::Mat(1, histogram_msg->histogram.size(), CV_32FC1);
    for (size_t i = 0; i < histogram_msg->histogram.size(); i++) {
      histogram_.at<float>(0, i) = histogram_msg->histogram[i];
    }
    normalizeHistogram(histogram_);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ColorHistogramLabelMatch, nodelet::Nodelet);
