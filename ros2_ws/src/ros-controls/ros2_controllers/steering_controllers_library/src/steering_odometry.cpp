// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: dr. sc. Tomislav Petkovic
 * Author: Dr. Ing. Denis Stogl
 */

#include "steering_controllers_library/steering_odometry.hpp"

#include <cmath>
#include <iostream>

namespace steering_odometry
{
SteeringOdometry::SteeringOdometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_track_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  traction_wheel_old_pos_(0.0),
  traction_right_wheel_old_pos_(0.0),
  traction_left_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_acc_(velocity_rolling_window_size),
  angular_acc_(velocity_rolling_window_size)
{
}

void SteeringOdometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  reset_accumulators();
  timestamp_ = time;
}

bool SteeringOdometry::update_odometry(
  const double linear_velocity, const double angular_velocity, const double dt)
{
  /// Integrate odometry:
  integrate_fk(linear_velocity, angular_velocity, dt);

  /// We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_.accumulate(linear_velocity);
  angular_acc_.accumulate(angular_velocity);

  linear_ = linear_acc_.getRollingMean();
  angular_ = angular_acc_.getRollingMean();

  return true;
}

bool SteeringOdometry::update_from_position(
  const double traction_wheel_pos, const double steer_pos, const double dt)
{
  const double traction_wheel_est_pos_diff = traction_wheel_pos - traction_wheel_old_pos_;

  /// Update old position with current:
  traction_wheel_old_pos_ = traction_wheel_pos;

  return update_from_velocity(traction_wheel_est_pos_diff / dt, steer_pos, dt);
}

bool SteeringOdometry::update_from_position(
  const double traction_right_wheel_pos, const double traction_left_wheel_pos,
  const double steer_pos, const double dt)
{
  const double traction_right_wheel_est_pos_diff =
    traction_right_wheel_pos - traction_right_wheel_old_pos_;
  const double traction_left_wheel_est_pos_diff =
    traction_left_wheel_pos - traction_left_wheel_old_pos_;

  /// Update old position with current:
  traction_right_wheel_old_pos_ = traction_right_wheel_pos;
  traction_left_wheel_old_pos_ = traction_left_wheel_pos;

  return update_from_velocity(
    traction_right_wheel_est_pos_diff / dt, traction_left_wheel_est_pos_diff / dt, steer_pos, dt);
}

bool SteeringOdometry::update_from_position(
  const double traction_right_wheel_pos, const double traction_left_wheel_pos,
  const double right_steer_pos, const double left_steer_pos, const double dt)
{
  const double traction_right_wheel_est_pos_diff =
    traction_right_wheel_pos - traction_right_wheel_old_pos_;
  const double traction_left_wheel_est_pos_diff =
    traction_left_wheel_pos - traction_left_wheel_old_pos_;

  /// Update old position with current:
  traction_right_wheel_old_pos_ = traction_right_wheel_pos;
  traction_left_wheel_old_pos_ = traction_left_wheel_pos;

  return update_from_velocity(
    traction_right_wheel_est_pos_diff / dt, traction_left_wheel_est_pos_diff / dt, right_steer_pos,
    left_steer_pos, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double traction_wheel_vel, const double steer_pos, const double dt)
{
  steer_pos_ = steer_pos;
  double linear_velocity = traction_wheel_vel * wheel_radius_;
  const double angular_velocity = tan(steer_pos) * linear_velocity / wheelbase_;

  return update_odometry(linear_velocity, angular_velocity, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double right_traction_wheel_vel, const double left_traction_wheel_vel,
  const double steer_pos, const double dt)
{
  double linear_velocity =
    (right_traction_wheel_vel + left_traction_wheel_vel) * wheel_radius_ * 0.5;
  steer_pos_ = steer_pos;

  const double angular_velocity = tan(steer_pos_) * linear_velocity / wheelbase_;

  return update_odometry(linear_velocity, angular_velocity, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double right_traction_wheel_vel, const double left_traction_wheel_vel,
  const double right_steer_pos, const double left_steer_pos, const double dt)
{
  steer_pos_ = (right_steer_pos + left_steer_pos) * 0.5;
  double linear_velocity =
    (right_traction_wheel_vel + left_traction_wheel_vel) * wheel_radius_ * 0.5;
  const double angular_velocity = steer_pos_ * linear_velocity / wheelbase_;

  return update_odometry(linear_velocity, angular_velocity, dt);
}

void SteeringOdometry::update_open_loop(const double v_bx, const double omega_bz, const double dt)
{
  /// Save last linear and angular velocity:
  linear_ = v_bx;
  angular_ = omega_bz;

  /// Integrate odometry:
  integrate_fk(v_bx, omega_bz, dt);
}

void SteeringOdometry::set_wheel_params(double wheel_radius, double wheelbase, double wheel_track)
{
  wheel_radius_ = wheel_radius;
  wheelbase_ = wheelbase;
  wheel_track_ = wheel_track;
}

void SteeringOdometry::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  reset_accumulators();
}

void SteeringOdometry::set_odometry_type(const unsigned int type) { config_type_ = type; }

double SteeringOdometry::convert_twist_to_steering_angle(double v_bx, double omega_bz)
{
  if (omega_bz == 0 || v_bx == 0)
  {
    return 0;
  }
  return std::atan(omega_bz * wheelbase_ / v_bx);
}

std::tuple<std::vector<double>, std::vector<double>> SteeringOdometry::get_commands(
  const double v_bx, const double omega_bz)
{
  // desired wheel speed and steering angle of the middle of traction and steering axis
  double Ws, phi;

  if (v_bx == 0 && omega_bz != 0)
  {
    // TODO(anyone) would be only valid if traction is on the steering axis -> tricycle_controller
    phi = omega_bz > 0 ? M_PI_2 : -M_PI_2;
    Ws = abs(omega_bz) * wheelbase_ / wheel_radius_;
  }
  else
  {
    phi = SteeringOdometry::convert_twist_to_steering_angle(v_bx, omega_bz);
    Ws = v_bx / (wheel_radius_ * std::cos(steer_pos_));
  }

  if (config_type_ == BICYCLE_CONFIG)
  {
    std::vector<double> traction_commands = {Ws};
    std::vector<double> steering_commands = {phi};
    return std::make_tuple(traction_commands, steering_commands);
  }
  else if (config_type_ == TRICYCLE_CONFIG)
  {
    std::vector<double> traction_commands;
    std::vector<double> steering_commands;
    if (fabs(steer_pos_) < 1e-6)
    {
      traction_commands = {Ws, Ws};
    }
    else
    {
      const double turning_radius = wheelbase_ / std::tan(steer_pos_);
      const double Wr = Ws * (turning_radius + wheel_track_ * 0.5) / turning_radius;
      const double Wl = Ws * (turning_radius - wheel_track_ * 0.5) / turning_radius;
      traction_commands = {Wr, Wl};
    }
    steering_commands = {phi};
    return std::make_tuple(traction_commands, steering_commands);
  }
  else if (config_type_ == ACKERMANN_CONFIG)
  {
    std::vector<double> traction_commands;
    std::vector<double> steering_commands;
    if (fabs(steer_pos_) < 1e-6)
    {
      traction_commands = {Ws, Ws};
      steering_commands = {phi, phi};
    }
    else
    {
      const double turning_radius = wheelbase_ / std::tan(steer_pos_);
      const double Wr = Ws * (turning_radius + wheel_track_ * 0.5) / turning_radius;
      const double Wl = Ws * (turning_radius - wheel_track_ * 0.5) / turning_radius;
      traction_commands = {Wr, Wl};

      const double numerator = 2 * wheelbase_ * std::sin(phi);
      const double denominator_first_member = 2 * wheelbase_ * std::cos(phi);
      const double denominator_second_member = wheel_track_ * std::sin(phi);

      const double alpha_r =
        std::atan2(numerator, denominator_first_member + denominator_second_member);
      const double alpha_l =
        std::atan2(numerator, denominator_first_member - denominator_second_member);
      steering_commands = {alpha_r, alpha_l};
    }
    return std::make_tuple(traction_commands, steering_commands);
  }
  else
  {
    throw std::runtime_error("Config not implemented");
  }
}

void SteeringOdometry::reset_odometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  reset_accumulators();
}

void SteeringOdometry::integrate_runge_kutta_2(
  const double v_bx, const double omega_bz, const double dt)
{
  // Compute intermediate value of the heading
  const double theta_mid = heading_ + omega_bz * 0.5 * dt;

  // Use the intermediate values to update the state
  x_ += v_bx * cos(theta_mid) * dt;
  y_ += v_bx * sin(theta_mid) * dt;
  heading_ += omega_bz * dt;
}

void SteeringOdometry::integrate_fk(const double v_bx, const double omega_bz, const double dt)
{
  const double delta_x_b = v_bx * dt;
  const double delta_theta = omega_bz * dt;

  if (fabs(delta_theta) < 1e-6)
  {
    /// Runge-Kutta 2nd Order (should solve problems when omega_bz is zero):
    integrate_runge_kutta_2(v_bx, omega_bz, dt);
  }
  else
  {
    /// Exact integration
    const double heading_old = heading_;
    const double R = delta_x_b / delta_theta;
    heading_ += delta_theta;
    x_ += R * (sin(heading_) - sin(heading_old));
    y_ += -R * (cos(heading_) - cos(heading_old));
  }
}

void SteeringOdometry::reset_accumulators()
{
  linear_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  angular_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
}

}  // namespace steering_odometry
