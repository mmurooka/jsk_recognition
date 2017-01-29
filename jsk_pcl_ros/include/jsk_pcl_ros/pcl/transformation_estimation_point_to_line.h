/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_LINE_H_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_LINE_H_

#include <cmath>

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationPointToLine uses Levenberg Marquardt optimization to find the
      * transformation that minimizes the point-to-line distance between the given correspondences.
      *
      * \author Masaki Murooka
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class TransformationEstimationPointToLine : public TransformationEstimationLM<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<TransformationEstimationPointToLine<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const TransformationEstimationPointToLine<PointSource, PointTarget, Scalar> > ConstPtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef PointIndices::Ptr PointIndicesPtr;
        typedef PointIndices::ConstPtr PointIndicesConstPtr;

        typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
        typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

        TransformationEstimationPointToLine () {};
      virtual ~TransformationEstimationPointToLine () {};

      protected:
        virtual Scalar
        computeDistance (const PointSource &p_src, const PointTarget &p_tgt) const
        {
          // Compute the point-to-line distance
          Vector4 p_src_vec (p_src.x, p_src.y, p_src.z, 0);
          return computeDistance (p_src_vec, p_tgt);
        }

      /**
       *  \brief compute distance between point and line
       *  \param p_src  point in source
       *  \param p_tgt  edge in target
       *                x,y,z represents start point and norm_x,y,z represents vector from start point to end point.
       *                edge length is not considered if consider_edge_length is false.
       */
        virtual Scalar
        computeDistance (const Vector4 &p_src, const PointTarget &p_tgt) const
        {
          bool consider_edge_length = true;

          // Compute the point-to-line distance
          Vector3 dir (p_tgt.normal_x, p_tgt.normal_y, p_tgt.normal_z);
          Vector3 p_start (p_tgt.x, p_tgt.y, p_tgt.z);
          Vector3 p_end (p_start + dir);
          double l = dir.norm();
          Vector3 q (p_src(0), p_src(1), p_src(2));

          Vector3 e (q - p_start);
          dir.normalize();
          double foot_dist = dir.transpose () * e;
          double dist;
          if (consider_edge_length && foot_dist < 0) {
            dist = (p_start - q).norm();
          } else if (consider_edge_length && foot_dist > l) {
            dist = (p_end - q).norm();
          } else {
            dist = pow(fabs(e.transpose () * e - foot_dist * foot_dist), 0.5);
          }
          return dist;
        }

    };
  }
}

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_LINE_H_ */

