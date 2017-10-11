/*
 Copyright (c) 2017, Inkyu Sa, Mina Kamel, Michael Burri, ASL, ETH Zurich, Switzerland

 You can contact the author at <inkyu.sa@mavt.ethz.ch> or <enddl22@gmail.com>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iomanip>
#include <iostream>
#include <sstream>

#include <librealsense/rsutil.h>

#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <realsense_ros_simple/helper.h>

std::ostream& operator <<(std::ostream& stream, const rs::extrinsics& e)
{
  stream << "rotation: " << std::endl << std::setprecision(5) << std::fixed;
  for (int r = 0; r < 3; ++r)
  {
    stream << "\t";
    for (int c = 0; c < 3; ++c)
    {
      stream << std::setw(8) << e.rotation[c * 3 + r] << " ";
    }
    stream << std::endl;
  }
  stream << "translation: " << std::endl << std::setprecision(5) << std::fixed << "\t";
  for (int i = 0; i < 3; ++i)
  {
    stream << std::setw(8) << e.translation[i] << " ";
  }
  stream << std::endl;
}

namespace realsense_ros_simple {

void invertExtrinsic(const rs::extrinsics& _in, rs::extrinsics* out)
{
  rs::extrinsics in = _in; // TODO: detect aliasing and only copy in that case.
  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
    {
      out->rotation[r * 3 + c] = in.rotation[c * 3 + r];
    }
  }

  const float* R = out->rotation;
  const float* t = in.translation;

  out->translation[0] = -(R[0] * t[0] + R[3] * t[1] + R[6] * t[2]);
  out->translation[1] = -(R[1] * t[0] + R[4] * t[1] + R[7] * t[2]);
  out->translation[2] = -(R[2] * t[0] + R[5] * t[1] + R[8] * t[2]);
}

geometry_msgs::TransformStamped rsExtrinsicToTf(const rs::extrinsics& e, const ros::Time& stamp, const std::string& parent, const std::string& child){
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;

  const float* R = e.rotation;
  tf2::Matrix3x3 tmpR(R[0], R[3], R[6],
                      R[1], R[4], R[7],
                      R[2], R[5], R[8]);

  tf2::Quaternion q;
  tmpR.getRotation(q);
  msg.transform.rotation.w = q.getW();
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();

  msg.transform.translation.x = e.translation[0];
  msg.transform.translation.y = e.translation[1];
  msg.transform.translation.z = e.translation[2];

  return msg;
}

void rsCalibrationToCameraInfoMsg(const rs::intrinsics& intrinsics,
                                           const rs::extrinsics& extrinsics,
                                           sensor_msgs::CameraInfo* camera_info)
{
  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for details.

  camera_info->width = intrinsics.width;
  camera_info->height = intrinsics.height;

  camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  static_assert(sizeof(intrinsics.coeffs) == 5*sizeof(float), "intrinsic parameters size mismatch");
  for(int i=0; i<5; ++i){
    camera_info->D.push_back(intrinsics.coeffs[i]);
  }

  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
    {
      camera_info->R[r * 3 + c] = extrinsics.rotation[c * 3 + r];
    }
  }

  camera_info->K.assign(0.0);
  camera_info->K[0] = intrinsics.fx;
  camera_info->K[2] = intrinsics.ppx;
  camera_info->K[4] = intrinsics.fy;
  camera_info->K[5] = intrinsics.ppy;
  camera_info->K[8] = 1.0;

  camera_info->P.assign(0.0);
  camera_info->P[0] = intrinsics.fx;
  camera_info->P[2] = intrinsics.ppx;
  camera_info->P[5] = intrinsics.fy;
  camera_info->P[6] = intrinsics.ppy;
  camera_info->P[10] = 1.0;

  rs::extrinsics T_inv;
  invertExtrinsic(extrinsics, &T_inv);
  camera_info->P[3] = T_inv.translation[0];
  camera_info->P[7] = T_inv.translation[1];
  camera_info->P[11] = T_inv.translation[2];

  camera_info->R[0] = 1.0;
  camera_info->R[1] = 0.0;
  camera_info->R[2] = 0.0;
  camera_info->R[3] = 0.0;
  camera_info->R[4] = 1.0;
  camera_info->R[5] = 0.0;
  camera_info->R[6] = 0.0;
  camera_info->R[7] = 0.0;
  camera_info->R[8] = 1.0;
}

void loadDepthParameters(const ros::NodeHandle& nh, rs::option options[10], double values[10]){
  // from rsutil.h
  static const rs_option depth_control_options[10] = {
      RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT,
      RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT,
      RS_OPTION_R200_DEPTH_CONTROL_MEDIAN_THRESHOLD,
      RS_OPTION_R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD,
      RS_OPTION_R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD,
      RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD,
      RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_DIFFERENCE_THRESHOLD,
      RS_OPTION_R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD,
      RS_OPTION_R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD,
      RS_OPTION_R200_DEPTH_CONTROL_LR_THRESHOLD
  };
  static double depth_control_presets[6][10] = {
      {5, 5, 192,  1,  512, 6, 24, 27,  7,   24}, /* (DEFAULT)   Default settings on chip. Similiar to the medium setting and best for outdoors. */
      {5, 5,   0,  0, 1023, 0,  0,  0,  0, 2047}, /* (OFF)       Disable almost all hardware-based outlier removal */
      {5, 5, 115,  1,  512, 6, 18, 25,  3,   24}, /* (LOW)       Provide a depthmap with a lower number of outliers removed, which has minimal false negatives. */
      {5, 5, 185,  5,  505, 6, 35, 45, 45,   14}, /* (MEDIUM)    Provide a depthmap with a medium number of outliers removed, which has balanced approach. */
      {5, 5, 175, 24,  430, 6, 48, 47, 24,   12}, /* (OPTIMIZED) Provide a depthmap with a medium/high number of outliers removed. Derived from an optimization function. */
      {5, 5, 235, 27,  420, 8, 80, 70, 90,   12}, /* (HIGH)      Provide a depthmap with a higher number of outliers removed, which has minimal false positives. */
  };

  int preset;
  bool havePreset = nh.getParam("depth/control/preset", preset);
  if (havePreset)
  {
    ROS_INFO("Having depth control preset %d -- ignoring individual parameters", preset);
    if (preset >= 6)
    {
      ROS_WARN("preset (%d) >= 6, choosing default depth parameters", preset);
      preset = 0;
    }
    for (int i = 0; i < 10; ++i)
    {
      values[i] = depth_control_presets[preset][i];
    }

    for (int i = 0; i < 10; ++i)
    {
      options[i] = (rs::option) depth_control_options[i];
    }
  } else
  {

    ROS_INFO("Loading individual depth control parameters");

    int _values[10];
    nh.param("depth/control/estimate_median_decrement", _values[0], 5);
    nh.param("depth/control/estimate_median_increment", _values[1], 5);
    nh.param("depth/control/median_threshold", _values[2], 235);
    nh.param("depth/control/score_minimum_threshold", _values[3], 27);
    nh.param("depth/control/score_maximum_threshold", _values[4], 420);
    nh.param("depth/control/texture_count_threshold", _values[5], 8);
    nh.param("depth/control/texture_difference_threshold", _values[6], 80);
    nh.param("depth/control/second_peak_threshold", _values[7], 70);
    nh.param("depth/control/neighbor_threshold", _values[8], 90);
    nh.param("depth/control/lr_threshold", _values[9], 12);

    for (int i = 0; i < 10; ++i)
    {
      options[i] = (rs::option) depth_control_options[i];
      values[i] = static_cast<double>(_values[i]);
    }
  }

  std::stringstream ss;
  for (int i = 0; i < 10; ++i)
  {
    ss << "\t" << options[i] << ": " << static_cast<int>(values[i]) << std::endl;
  }
  ROS_INFO_STREAM("Depth control parameters:" << std::endl<< ss.str());
}

}
