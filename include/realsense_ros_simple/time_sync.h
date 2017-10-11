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

#include <iostream>

#include <Eigen/Dense>
#include <ros/ros.h>

#ifndef INCLUDE_REALSENSE_ROS_TIME_SYNC_H_
#define INCLUDE_REALSENSE_ROS_TIME_SYNC_H_

namespace realsense_ros_simple {

class TimeSyncFilter {
 public:
  static constexpr double kSigmaInitOffset = 2e-3;
  static constexpr double kSigmaInitSkew = 1e-3;
  static constexpr double kSigmaMeasurementTimeOffset = 2e-3;
  static constexpr double kSigmaSkew = 2e-6;
  static constexpr double kSigmaOffset = 1e-5;
  static constexpr double kUpdateRate = 0.1; // (1/sec)


  TimeSyncFilter();
  ~TimeSyncFilter() {};
  double getLocalTimestamp(double device_time);
  void updateFilter(double device_time, double local_time);
  void print();
  void reset();
 private:
  void initialize(double device_time, double local_time);
  bool isOutlier(double device_time, double local_time);

  Eigen::Vector2d x_;
  Eigen::Matrix2d P_;
  Eigen::Matrix2d Q_;
  double R_;
  Eigen::Matrix<double, 1, 2> H_;
  bool isInitialized_;
  double lastUpdateDeviceTime_;
  double dt_;
};

}

#endif /* INCLUDE_REALSENSE_ROS_TIME_SYNC_H_ */
