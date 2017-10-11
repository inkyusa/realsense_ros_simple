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

#ifndef INCLUDE_REALSENSE_ROS_ZR300_H_
#define INCLUDE_REALSENSE_ROS_ZR300_H_

#include <deque>
#include <mutex>

#include <librealsense/rs.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_publisher.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "realsense_ros_simple/time_sync.h"

#include <ze/imu/imu_buffer.hpp>


namespace realsense_ros_simple {

class ZR300 {
 public:
  ZR300(ros::NodeHandle nh, ros::NodeHandle privateNh,
        const std::string& frameId = "");
  ~ZR300();
  bool start();
  void stop();
  void publishStaticTransforms();

  static constexpr auto kBaseFrameName = "realsense_base";
  static constexpr auto kInfraredFrameName = "infrared";
  static constexpr auto kFisheyeFrameName = "fisheye";
  static constexpr auto kImuFrameName = "imu";

  static constexpr double kTimeshifCamImu =
      0.00972;  // Delay of the camera timestamps w.r.t. the IMU
  // value provided by KalibR, but not sure if the same on all devices.

 private:
  static constexpr double kImuTimestampToSeconds = 1.0e-3;
  static constexpr size_t kTimestampMaxQueueSize = 100;

  struct rgb_color {
    uint8_t r, g, b;
  };
  struct hsv_color {
    uint8_t h, s, v;
  };

  bool enableStreams();
  bool configureStaticOptions();

  void motionCallback(rs::motion_data entry);
  void timestampCallback(rs::timestamp_data entry);
  // Fills up the image queues with new frames.
  void frameCallback(rs::frame frame);

  // Checks the image queues and publishes if timestamps are already available.
  void checkTimestampsAndPublish();

  //void buildPointcloud();

  bool findTimestamp(unsigned short frameNumber, rs_event_source source,
                     int* timestampImu, ros::Time* timestamp);

  void filterDepth(cv::Mat* depth_image);
  bool readCalibrations();

  rs::context zr300Context_;
  rs::device* zr300Device_;

  mutable std::mutex timestampMutex_;
  std::deque<rs::timestamp_data> timestampQueue_;
  double clockSkew_;

  bool gotFirstTimestamp_;

  ros::NodeHandle nh_, privateNh_;
  std::string framePrefix_;

  //exclusively for calibration purposes
  ros::Publisher pubAccel_;
  ros::Publisher pubGyro_;
  bool publish_imu_individually_;

  //exclusively for compensation purposes
  typedef ze::ImuBufferLinear2000 buffer_t;
  //typedef ze::ImuBufferNearest2000 buffer_t; //for assessing the gain through interpolation
  buffer_t::Ptr imu_buffer_;
  ros::Publisher imu_compensated_;
  std::deque<int64_t> gyro_timestamps_;

  ros::Publisher pubImu_;
  image_transport::CameraPublisher pubFisheye_;

  rs::extrinsics T_infrared_fisheye_;
  rs::intrinsics intrinsicFisheye_;
  sensor_msgs::CameraInfoPtr infoFisheye_;
  sensor_msgs::ImagePtr image_fisheye_;

  // These are the sequence numbers (frame IDs) of the above images for
  // timestamp resolution and publishing.
  int frame_id_fisheye_;

  rs::motion_intrinsics intrinsicsImu_;

  tf2_ros::StaticTransformBroadcaster extrinsicBroadcaster_;

  double lastAcceleration_[3];

  TimeSyncFilter timeSynchronizer_;

  // fisheye options
  bool fisheye_enable_auto_exposure_;
  bool fisheye_enable_strobe_;
  double fisheye_exposure_ms_;
  double fisheye_gain_;
  int fisheye_subsample_factor_;
  int fisheye_anti_flicker_rate_hz_;
  int fisheye_auto_exposure_mode_;
  int fisheye_auto_exposure_sample_rate_;
  int fisheye_auto_exposure_skip_frames_;
  std::mutex image_queue_mutex_;
};
}

#endif /* INCLUDE_REALSENSE_ROS_ZR300_H_ */
