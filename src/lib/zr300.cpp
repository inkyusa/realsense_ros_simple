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

#include <chrono>

#include <realsense_ros_simple/helper.h>
#include <realsense_ros_simple/zr300.h>

#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

namespace realsense_ros_simple {

ZR300::ZR300(ros::NodeHandle nh, ros::NodeHandle privateNh,
    const std::string& frameId)
: zr300Device_(nullptr),
  nh_(nh),
  privateNh_(privateNh),
  gotFirstTimestamp_(false),
  publish_imu_individually_(false) {
  framePrefix_ = frameId.empty() ? "" : (frameId + "/");

  auto advertiseCamera =
      [nh](const std::string& name) -> image_transport::CameraPublisher {
    ros::NodeHandle _nh(nh, name);
    image_transport::ImageTransport it(_nh);
    return it.advertiseCamera(name, 1);
  };
  pubFisheye_ = advertiseCamera(kFisheyeFrameName);
  pubImu_ = nh.advertise<sensor_msgs::Imu>("imu/raw", 1);
  privateNh_.param("imu/publish_sensors_inidividually", publish_imu_individually_,
      false);
  if (publish_imu_individually_) {
    pubAccel_ = nh.advertise<sensor_msgs::Imu>("imu/accel_raw", 1);
    pubGyro_ = nh.advertise<sensor_msgs::Imu>("imu/gyro_raw", 1);
  }

  std::string imu_yaml;
  privateNh_.param("imu/intrinsics_yaml", imu_yaml, std::string());
  if (!imu_yaml.empty()) {
    imu_buffer_ = std::make_shared<buffer_t>(ze::ImuModel::loadFromYaml(imu_yaml));
    if (imu_buffer_) {
      imu_compensated_ = nh.advertise<sensor_msgs::Imu>("imu/compensated", 1);
    }
    else {
      ROS_ERROR("Failed to load IMU intrinsics from file %s.", imu_yaml.c_str());
    }
  }
}

ZR300::~ZR300() { stop(); }

bool ZR300::start() {
  rs::log_to_console(rs::log_severity::info);

  // Create a context object. This object owns the handles to all connected
  // realsense devices.
  rs::context ctx;

  ROS_INFO_STREAM("There are " << ctx.get_device_count()
      << " connected RealSense devices.");

  if (ctx.get_device_count() == 0) return EXIT_FAILURE;

  zr300Device_ = ctx.get_device(0);
  ROS_INFO("\nUsing device 0, an %s", zr300Device_->get_name());
  ROS_INFO("    Serial number: %s", zr300Device_->get_serial());
  ROS_INFO("    Firmware version: %s", zr300Device_->get_firmware_version());

  if (zr300Device_->supports(rs::capabilities::motion_events)) {
    zr300Device_->enable_motion_tracking(
        std::bind(&ZR300::motionCallback, this, std::placeholders::_1),
        std::bind(&ZR300::timestampCallback, this, std::placeholders::_1));
  } else {
    ROS_ERROR("motion module not supported");
    return false;
  }

  enableStreams();
  //configureStaticOptions();

  for (int i = (int)(rs::stream::depth); i <= (int)(rs::stream::fisheye); i++) {
    zr300Device_->set_frame_callback(
        (rs::stream)i,
        std::bind(&ZR300::frameCallback, this, std::placeholders::_1));
  }

  readCalibrations();
  publishStaticTransforms();

  zr300Device_->start(rs::source::all_sources);

  return true;
}

void ZR300::stop() {
  zr300Device_->stop(rs::source::all_sources);
  zr300Device_->disable_motion_tracking();
}

void ZR300::publishStaticTransforms() {
  std::vector<geometry_msgs::TransformStamped> transforms;
  ros::Time stamp = ros::Time::now();
  const std::string parent = framePrefix_ + kInfraredFrameName;
  transforms.push_back(rsExtrinsicToTf(T_infrared_fisheye_, stamp, parent,
      framePrefix_ + kFisheyeFrameName));
  extrinsicBroadcaster_.sendTransform(transforms);
}

bool ZR300::enableStreams() {
  int width, height, fps;
  bool enable;
  privateNh_.param("fisheye/width", width, 640);
  privateNh_.param("fisheye/height", height, 480);
  privateNh_.param("fisheye/fps", fps, 30);
  privateNh_.param("fisheye/enabled", enable, true);
  privateNh_.param("fisheye/enable_auto_exposure",
                   fisheye_enable_auto_exposure_, true);
  privateNh_.param("fisheye/exposure_ms", fisheye_exposure_ms_,
                   40.0);  // 40ms is minimum
  privateNh_.param("fisheye/gain", fisheye_gain_, 9.0);
  privateNh_.param("fisheye/subsample_factor", fisheye_subsample_factor_,
                   1);  // Don't publish all images.
  privateNh_.param("fisheye/enable_strobe", fisheye_enable_strobe_, true);
  privateNh_.param("fisheye/auto_exposure_mode", fisheye_auto_exposure_mode_, 0);
  privateNh_.param("fisheye/anti_flicker_rate_hz", fisheye_anti_flicker_rate_hz_, 50);
  privateNh_.param("fisheye/auto_exposure_sample_rate", fisheye_auto_exposure_sample_rate_, 1);
  privateNh_.param("fisheye/auto_exposure_skip_frames", fisheye_auto_exposure_skip_frames_, 2);

  if (enable) {
    zr300Device_->enable_stream(rs::stream::fisheye, width, height,
        rs::format::raw8, fps);
  }
  return true;
}

void ZR300::motionCallback(rs::motion_data entry) {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto sys_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  std::stringstream ss;
  ss << "Motion,\t host time " << sys_time
      << "\ttimestamp: " << std::setprecision(8)
  << (double)entry.timestamp_data.timestamp * kImuTimestampToSeconds
  << "\tsource: " << (rs::event)entry.timestamp_data.source_id
  << "\tframe_num: " << entry.timestamp_data.frame_number
  << "\tx: " << std::setprecision(5) << entry.axes[0]
                                                   << "\ty: " << entry.axes[1] << "\tz: " << entry.axes[2];
  // ROS_INFO_STREAM("" << ss.str() << "--" << entry.is_valid );
  if (/*entry.is_valid && */ (rs::event)entry.timestamp_data.source_id ==
      rs::event::event_imu_gyro) {
    if (pubImu_.getNumSubscribers() > 0) {
      double device_time =
          (double)entry.timestamp_data.timestamp * kImuTimestampToSeconds;
      //For backwards compatibility: compensate the offset only on raw IMU,
      //otherwise the offset is compensated for twice.
      double local_time = timeSynchronizer_.getLocalTimestamp(device_time) - kTimeshifCamImu;

      sensor_msgs::ImuPtr msg = boost::make_shared<sensor_msgs::Imu>();
      // msg->header.stamp = ros::Time::now();
      msg->header.stamp = ros::Time(local_time);
      msg->header.seq = entry.timestamp_data.frame_number;
      msg->header.frame_id = framePrefix_ + "/" + kImuFrameName;

      msg->angular_velocity.x = entry.axes[0];
      msg->angular_velocity.y = entry.axes[1];
      msg->angular_velocity.z = entry.axes[2];

      msg->linear_acceleration.x = lastAcceleration_[0];
      msg->linear_acceleration.y = lastAcceleration_[1];
      msg->linear_acceleration.z = lastAcceleration_[2];

      msg->orientation_covariance[0] = -1.0;  // No orientation estimate.

      pubImu_.publish(msg);
    }
  }

  if (/*entry.is_valid && */ (rs::event)entry.timestamp_data.source_id ==
      rs::event::event_imu_accel) {
    lastAcceleration_[0] = entry.axes[0];
    lastAcceleration_[1] = entry.axes[1];
    lastAcceleration_[2] = entry.axes[2];
  }

  if (publish_imu_individually_) {
    sensor_msgs::ImuPtr msg = boost::make_shared<sensor_msgs::Imu>();
    msg->header.stamp = ros::Time(
        static_cast<double>(entry.timestamp_data.timestamp)
        * kImuTimestampToSeconds);
    msg->header.seq = entry.timestamp_data.frame_number;
    msg->header.frame_id = framePrefix_ + "/" + kImuFrameName;
    msg->orientation_covariance[0] = -1.0;  // No orientation estimate.

    if (static_cast<rs::event>(entry.timestamp_data.source_id) ==
        rs::event::event_imu_accel) {
      msg->linear_acceleration.x = entry.axes[0];
      msg->linear_acceleration.y = entry.axes[1];
      msg->linear_acceleration.z = entry.axes[2];

      pubAccel_.publish(msg);
    }
    else if (static_cast<rs::event>(entry.timestamp_data.source_id) ==
        rs::event::event_imu_gyro) {
      msg->angular_velocity.x = entry.axes[0];
      msg->angular_velocity.y = entry.axes[1];
      msg->angular_velocity.z = entry.axes[2];

      pubGyro_.publish(msg);
    }
  }

  if (imu_buffer_) {
    double device_time =
        (double)entry.timestamp_data.timestamp * kImuTimestampToSeconds;
    double local_time = timeSynchronizer_.getLocalTimestamp(device_time);
    int64_t time_ns = static_cast<int64_t>(local_time * 1.e9);
    const Eigen::Vector3d measurement = (Eigen::Vector3d() << entry.axes[0],
        entry.axes[1], entry.axes[2]).finished();

    if (static_cast<rs::event>(entry.timestamp_data.source_id) ==
        rs::event::event_imu_gyro) {
      imu_buffer_->insertGyroscopeMeasurement(time_ns, measurement);
      gyro_timestamps_.push_back(time_ns);
    }
    else if (static_cast<rs::event>(entry.timestamp_data.source_id) ==
        rs::event::event_imu_accel) {
      imu_buffer_->insertAccelerometerMeasurement(time_ns, measurement);
    }

    while (!gyro_timestamps_.empty()) {
      const int64_t gyro_ts = gyro_timestamps_.front();

      int64_t ts_oldest;
      int64_t ts_newest;
      bool success_ts;
      std::tie(ts_oldest, ts_newest, success_ts) = imu_buffer_->
          getOldestAndNewestStamp();
      //The buffer seems to be empty.
      if (!success_ts) {
        break;
      }
      //The timestamp is too old, the corresponding measurements are not in the
      // buffer anymore.
      if (gyro_ts < ts_oldest) {
        gyro_timestamps_.pop_front();
        continue;
      }

      Eigen::Matrix<double,6,1> compensated_measurement;
      bool success_lookup = imu_buffer_->get(gyro_ts, compensated_measurement);
      if (!success_lookup) {
        break;
      }

      sensor_msgs::ImuPtr msg = boost::make_shared<sensor_msgs::Imu>();
      msg->header.stamp = ros::Time(gyro_ts * 1.e-9);
      msg->header.seq = entry.timestamp_data.frame_number;
      msg->header.frame_id = framePrefix_ + "/" + kImuFrameName;
      msg->orientation_covariance[0] = -1.0;  // No orientation estimate.
      msg->linear_acceleration.x = compensated_measurement(0);
      msg->linear_acceleration.y = compensated_measurement(1);
      msg->linear_acceleration.z = compensated_measurement(2);
      msg->angular_velocity.x = compensated_measurement(3);
      msg->angular_velocity.y = compensated_measurement(4);
      msg->angular_velocity.z = compensated_measurement(5);
      imu_compensated_.publish(msg);

      gyro_timestamps_.pop_front();
    }

  }
}

void ZR300::timestampCallback(rs::timestamp_data entry) {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto sys_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  double time_now = ros::Time::now().toSec();

  std::stringstream ss;
  ss << "TimeEvt, host time " << sys_time
      << "\ttimestamp: " << std::setprecision(8)
  << (double)entry.timestamp * kImuTimestampToSeconds
  << "\tsource: " << (rs::event)entry.source_id
  << "\tframe_num: " << entry.frame_number;
  // ROS_INFO_STREAM("" << ss.str());

  if (!gotFirstTimestamp_) {
    std::lock_guard<std::mutex> lock(timestampMutex_);
    // timeOffset_ = ros::Time::now() -
    // ros::Time(static_cast<double>(entry.timestamp)*kImuTimestampToSeconds);
    timeSynchronizer_.updateFilter(
        static_cast<double>(entry.timestamp) * kImuTimestampToSeconds,
        time_now);
    gotFirstTimestamp_ = true;
  }

  if ((rs::event)entry.source_id == rs::event::event_imu_depth_cam ||
      (rs::event)entry.source_id == rs::event::event_imu_motion_cam) {
    std::lock_guard<std::mutex> lock(timestampMutex_);
    timeSynchronizer_.updateFilter(
        static_cast<double>(entry.timestamp) * kImuTimestampToSeconds,
        time_now);
    timestampQueue_.push_back(entry);
    while (timestampQueue_.size() > kTimestampMaxQueueSize) {
      timestampQueue_.pop_front();
    }
  }
}

void ZR300::frameCallback(rs::frame frame) {
  checkTimestampsAndPublish();

  {
    //std::lock_guard<std::mutex> lock(image_queue_mutex_);

    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto sys_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    std::stringstream ss;
    ss << "Frame data,   time " << sys_time
       << "\ttimestamp: " << std::setprecision(8) << frame.get_timestamp()
       << "\tsource: " << std::setw(13) << frame.get_stream_type()
       << "\tframe_num: " << frame.get_frame_number()
       << "\tdomain: " << (int)frame.get_frame_timestamp_domain();
    // ROS_INFO_STREAM("" << ss.str());

    rs::stream st = frame.get_stream_type();
    if (st == rs::stream::fisheye) {
      if (frame.get_frame_number() % fisheye_subsample_factor_ != 0) {
        return;
      }
      if (true /*pubFisheye_.getNumSubscribers() > 0 && gotFirstTimestamp_*/) {
        if (image_fisheye_) {
          ROS_WARN("Dropping a fisheye image!");
        } else {
          image_fisheye_.reset(new sensor_msgs::Image());
        }
        frame_id_fisheye_ = frame.get_frame_number();
        image_fisheye_->header.frame_id = framePrefix_ + kFisheyeFrameName;
        image_fisheye_->height = frame.get_height();
        image_fisheye_->width = frame.get_width();
        image_fisheye_->step = image_fisheye_->width;
        image_fisheye_->encoding = sensor_msgs::image_encodings::MONO8;
        const size_t size = image_fisheye_->height * image_fisheye_->step;
        image_fisheye_->data.resize(size);
        memcpy(image_fisheye_->data.data(), frame.get_data(), size);
      }
    }
  }

  checkTimestampsAndPublish();
}

void ZR300::checkTimestampsAndPublish() {
  // Check all queues for available timestamps and publish.
  // If there are no subscribers, the queues will remain empty.
  //std::lock_guard<std::mutex> lock(image_queue_mutex_);

  ros::Time local_timestamp = ros::Time::now();
  if (image_fisheye_ &&
      findTimestamp(frame_id_fisheye_, RS_EVENT_IMU_MOTION_CAM, nullptr,
                    &local_timestamp)) {
    std::stringstream ss;
    ss << "Publishing fisheye: "
       << "timestamp: " << std::setprecision(8) << local_timestamp.toSec()
       << "\tframe_num: " << frame_id_fisheye_;
    // ROS_INFO_STREAM("" << ss.str());

    image_fisheye_->header.stamp = local_timestamp;
    infoFisheye_->header = image_fisheye_->header;
    pubFisheye_.publish(image_fisheye_, infoFisheye_);

    frame_id_fisheye_ = -1;
    image_fisheye_.reset();
  }
}

bool ZR300::findTimestamp(unsigned short frameNumber, rs_event_source source,
    int* timestampImu, ros::Time* timestamp) {
  std::lock_guard<std::mutex> lock(timestampMutex_);

  for (auto ts : timestampQueue_) {
    if (ts.source_id == source && ts.frame_number == frameNumber) {
      if (timestampImu) {
        *timestampImu = ts.timestamp;
      }
      if (timestamp) {
        double device_time =
            static_cast<double>(ts.timestamp) * kImuTimestampToSeconds;
        ros::Time local_time;
        double local_timestamp;
        local_timestamp = timeSynchronizer_.getLocalTimestamp(device_time);
        if (!publish_imu_individually_) {
          local_time.fromSec(local_timestamp);
        }
        else {
          local_time.fromSec(device_time);
        }
        *timestamp = local_time;
      }
      return true;
    }
  }
  return false;
}

bool ZR300::readCalibrations() {
  bool success = true;
  try {
    T_infrared_fisheye_ =
        zr300Device_->get_motion_extrinsics_from(rs::stream::infrared);
    invertExtrinsic(T_infrared_fisheye_,
                    &T_infrared_fisheye_);  // librealsense sees it vice versa
  } catch (rs::error& e) {
    ROS_ERROR_STREAM("Unable to get fisheye extrinsics: " << e.what());
    for (int i = 0; i < 9; ++i) {
      T_infrared_fisheye_.rotation[i] = 0.f;
    }
    T_infrared_fisheye_.rotation[0] = 1.f;
    T_infrared_fisheye_.rotation[4] = 1.f;
    T_infrared_fisheye_.rotation[8] = 1.f;
    T_infrared_fisheye_.translation[0] = -0.03f;
    T_infrared_fisheye_.translation[1] = 0.0f;
    T_infrared_fisheye_.translation[2] = 0.0f;

    success = false;
  }
  ROS_INFO_STREAM("\n"
                  << "FISHEYE"
                  << " extrinsic: " << T_infrared_fisheye_);

  try {
    intrinsicFisheye_ =
        zr300Device_->get_stream_intrinsics(rs::stream::fisheye);
  } catch (rs::error& e) {
    ROS_ERROR_STREAM("Unable to get fisheye intrinsics: " << e.what());
    success = false;
  }

  try {
    intrinsicsImu_ = zr300Device_->get_motion_intrinsics();
  } catch (rs::error& e) {
    ROS_ERROR_STREAM("Unable to get IMU intrinsics: " << e.what());
    auto setDefault = [](rs_motion_device_intrinsic* i) {
      /* Scale X        cross axis        cross axis      Bias X */
      /* cross axis     Scale Y           cross axis      Bias Y */
      /* cross axis     cross axis        Scale Z         Bias Z */
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) i->data[r][c] = 0.f;

      i->data[0][0] = 1.f;
      i->data[1][1] = 1.f;
      i->data[2][2] = 1.f;

      for (int r = 0; r < 3; ++r) {
        i->noise_variances[r] = 0.f;  // TODO
        i->bias_variances[r] = 0.f;
      }
    };

    setDefault(&intrinsicsImu_.gyro);
    setDefault(&intrinsicsImu_.acc);

    success = false;
  }

  rs::extrinsics T_eye;
  for (int i = 0; i < 9; ++i) {
    T_eye.rotation[i] = 0.f;
  }
  T_eye.rotation[0] = 1.f;
  T_eye.rotation[4] = 1.f;
  T_eye.rotation[8] = 1.f;
  T_eye.translation[0] = 0.f;
  T_eye.translation[1] = 0.f;
  T_eye.translation[2] = 0.f;

  infoFisheye_.reset(new sensor_msgs::CameraInfo());
  rsCalibrationToCameraInfoMsg(intrinsicFisheye_, T_infrared_fisheye_,
                               infoFisheye_.get());
  return success;
}
}
