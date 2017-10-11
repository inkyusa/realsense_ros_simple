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

#include "realsense_ros_simple/time_sync.h"

namespace realsense_ros_simple {

TimeSyncFilter::TimeSyncFilter():isInitialized_(false) {

}

double TimeSyncFilter::getLocalTimestamp(double device_time) {
  if(!isInitialized_) {
    std::cout << "[WARN]: Timesync filter not initialized yet! Hack initializing now.";
    double local_time = ros::Time::now().toSec();
    initialize(device_time, local_time);
  }
  double dt = device_time - lastUpdateDeviceTime_;
  return device_time + x_(0) + dt * x_(1);
}

void TimeSyncFilter::print() {
  std::cout << "offset: " << x_(0) << "skew: " << x_(1) << "dt" << dt_ << std::endl;
  // std::cout << P_<< std::endl;
}

void TimeSyncFilter::reset() {
  isInitialized_ = 0;
}

void TimeSyncFilter::updateFilter(double device_time, double local_time) {
  if(!isInitialized_) {
    initialize(device_time, local_time);
    return;
  }

  double dt = device_time - lastUpdateDeviceTime_;

  if(dt < kUpdateRate)
    return;

  dt_=dt;
  Eigen::Matrix2d F;
  F << 1, dt_, 0, 1;

  // Prediction
  Eigen::Vector2d x_pred = F * x_;

  // check for outlier
  double measurement_residual = local_time - device_time - H_ * x_pred;

  if(measurement_residual > 2*kSigmaMeasurementTimeOffset) {
    // std::cout << "Timesync outlier" << std::endl;
    return;
  }


  P_ = F * P_ * F.transpose() + dt_ * Q_;

  // Update
  double S = H_ * P_ * H_.transpose() + R_;

  Eigen::Vector2d K;
  K = P_ * H_.transpose() * (1 / S);

  x_ = x_pred + K * measurement_residual;
  P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;

  lastUpdateDeviceTime_ = device_time;

  // print();
}


void TimeSyncFilter::initialize(double device_time, double local_time){
  x_.setZero();
  x_[0] = local_time - device_time;

  P_.setZero();
  P_(0,0) = kSigmaInitOffset * kSigmaInitOffset;
  P_(1,1) = kSigmaInitSkew * kSigmaInitSkew;

  Q_.setZero();
  Q_(0,0) = kSigmaOffset * kSigmaOffset;
  Q_(1,1) = kSigmaSkew * kSigmaSkew;

  R_ = kSigmaMeasurementTimeOffset * kSigmaMeasurementTimeOffset;

  H_.setZero();
  H_(0,0) = 1;

  lastUpdateDeviceTime_ = device_time;
  isInitialized_ = true;
}

}
