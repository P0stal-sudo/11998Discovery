/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"

using namespace ez;

// Sets and gets
void Drive::odom_x_set(double x) {
  odom_current.x = x;
  l_pose.x = x;
  r_pose.x = x;
  central_pose.x = x;
}
void Drive::odom_y_set(double y) {
  odom_current.y = y;
  l_pose.y = y;
  r_pose.y = y;
  central_pose.y = y;
}
void Drive::odom_theta_set(double a) {
  drive_angle_set(a);
  central_pose.theta = a;
}
void Drive::odom_x_set(okapi::QLength p_x) { odom_x_set(p_x.convert(okapi::inch)); }
void Drive::odom_y_set(okapi::QLength p_y) { odom_y_set(p_y.convert(okapi::inch)); }
void Drive::odom_theta_set(okapi::QAngle p_a) { odom_theta_set(p_a.convert(okapi::degree)); }
void Drive::odom_reset() { odom_pose_set({0, 0, 0}); }
void Drive::drive_width_set(double input) {
  global_track_width = fabs(input);
  if (input != 0.0) {
    odom_ime_track_width_left = -(global_track_width / 2.0);
    odom_ime_track_width_right = (global_track_width / 2.0);
    return;
  }
  odom_ime_track_width_left = 0.0;
  odom_ime_track_width_right = 0.0;
}
void Drive::drive_width_set(okapi::QLength p_input) { drive_width_set(p_input.convert(okapi::inch)); }
double Drive::drive_width_get() { return global_track_width; }
void Drive::odom_enable(bool input) { odometry_enabled = input; }
bool Drive::odom_enabled() { return odometry_enabled; }
void Drive::odom_pose_set(united_pose itarget) { odom_pose_set(util::united_pose_to_pose(itarget)); }
void Drive::odom_pose_set(pose itarget) {
  odom_theta_set(itarget.theta);
  odom_x_set(itarget.x);
  odom_y_set(itarget.y);
}

double Drive::odom_x_get() { return odom_current.x; }
double Drive::odom_y_get() { return odom_current.y; }
double Drive::odom_theta_get() { return odom_current.theta; }
pose Drive::odom_pose_get() { return odom_current; }

std::pair<float, float> Drive::decide_vert_sensor(ez::tracking_wheel *tracker, bool is_tracker_enabled, float ime, float ime_track) {
  float current = ime;
  float track_width = ime_track;
  if (is_tracker_enabled) {
    current = tracker->get();
    track_width = tracker->distance_to_center_get();
  }

  return {current, track_width};
}

ez::pose Drive::solve_xy_vert(float p_track_width, float current_t, float delta_vert, float delta_t) {
  pose output = {0.0, 0.0, 0.0};

  // Figure out how far we've actually moved
  float local_x = delta_vert;
  float half_delta_t = 0.0;
  if (delta_t != 0) {
    half_delta_t = delta_t / 2.0;
    float i = sin(half_delta_t) * 2.0;
    local_x = (delta_vert / delta_t - p_track_width) * i;
  }

  float alpha = current_t - half_delta_t;
  float x = cos(alpha) * local_x;
  float y = sin(alpha) * local_x;

  // xy is calculated internally using math standard but translated to what's intuitive
  // where going forward from 0 degrees increases Y
  output.x = -y;
  output.y = x;

  return output;
}

ez::pose Drive::solve_xy_horiz(float p_track_width, float current_t, float delta_horiz, float delta_t) {
  pose output = {0.0, 0.0, 0.0};

  // Figure out how far we've actually moved
  float local_y = delta_horiz;
  float half_delta_t = 0.0;
  if (delta_t != 0) {
    half_delta_t = delta_t / 2.0;
    float i = sin(half_delta_t) * 2.0;
    local_y = (delta_horiz / delta_t + p_track_width) * i;
  }

  float alpha = current_t - half_delta_t;
  float x = -sin(alpha) * local_y;
  float y = cos(alpha) * local_y;

  // xy is calculated internally using math standard but translated to what's intuitive
  // where going forward from 0 degrees increases Y
  output.x = -y;
  output.y = x;

  return output;
}
// pose central_pose;
// Tracking based on https://wiki.purduesigbots.com/software/odometry
void Drive::ez_tracking_task() {
  // Don't let this function run if odom is disabled
  // and make sure all the "lasts" are 0
  if (!imu_calibration_complete || !odometry_enabled) {
    v_last = 0.0;
    h_last = 0.0;
    t_last = 0.0;
    l_last = 0.0;
    r_last = 0.0;
    return;
  }

  // Decide on using a horiz tracker vs not
  std::pair<float, float> h_cur_and_track = decide_vert_sensor(odom_back_tracker, odom_back_tracker_enabled);
  float h_current = h_cur_and_track.first;
  float h_track_width = h_cur_and_track.second;
  // Calculate velocity based on horiz value
  float h_ = h_current - h_last;
  h_last = h_current;

  // Decide on left ime vs left tracker
  std::pair<float, float> l_cur_and_track = decide_vert_sensor(odom_left_tracker, odom_left_tracker_enabled, drive_sensor_left(), odom_ime_track_width_left);
  float l_current = l_cur_and_track.first;
  float l_track_width = l_cur_and_track.second;
  // Calculate velocity based on left value
  float l_ = l_current - l_last;
  l_last = l_current;

  // Decide on right ime vs right tracker
  std::pair<float, float> r_cur_and_track = decide_vert_sensor(odom_right_tracker, odom_right_tracker_enabled, drive_sensor_right(), odom_ime_track_width_right);
  float r_current = r_cur_and_track.first;
  float r_track_width = r_cur_and_track.second;
  // Calculate velocity based on left value
  float r_ = r_current - r_last;
  r_last = r_current;

  // Angle and velocity
  float t_current = -ez::util::to_rad(drive_imu_get());  // negative for math standard
  float t_ = t_current - t_last;
  t_last = t_current;

  pose h_pose = solve_xy_horiz(h_track_width, t_current, h_, t_);
  pose l_pose_ = solve_xy_vert(l_track_width, t_current, l_, t_);
  pose r_pose_ = solve_xy_vert(r_track_width, t_current, r_, t_);

  r_pose.x += r_pose_.x;
  r_pose.y += r_pose_.y;
  r_pose.x += h_pose.x;
  r_pose.y += h_pose.y;

  l_pose.x += l_pose_.x;
  l_pose.y += l_pose_.y;
  l_pose.x += h_pose.x;
  l_pose.y += h_pose.y;

  double avg = l_ + r_;
  if (avg != 0.0)
    avg /= 2.0;
  pose central_pose_ = solve_xy_vert(0.0, t_current, avg, t_);
  central_pose.x += central_pose_.x;
  central_pose.y += central_pose_.y;

  odom_current.x = central_pose.x;
  odom_current.y = central_pose.y;
  // odom_current.x = (l_pose.x + r_pose.x) / 2.0;
  // odom_current.y = (l_pose.y + r_pose.y) / 2.0;

  /*
  // If there is a vert tracker, use it
  if (odom_left_tracker_enabled != odom_right_tracker_enabled) {
    if (odom_left_tracker_enabled) {
      odom_current.x = l_pose.x;
      odom_current.y = l_pose.y;
    } else if (odom_right_tracker_enabled) {
      odom_current.x = r_pose.x;
      odom_current.y = r_pose.y;
    }
  }
  // If both sides have a sensor (2 vert trackers or 0 trackers), let the user pick what side
  //  defaults to left
  else {
    if (odom_ime_use_left) {
      odom_current.x = l_pose.x;
      odom_current.y = l_pose.y;
    } else {
      odom_current.x = r_pose.x;
      odom_current.y = r_pose.y;
    }
  }
  */
  odom_current.theta = drive_imu_get();

  // printf("odom_ime_track_width_left %f   l_ %f   r_ %f   t_current %f\n", odom_ime_track_width_left, r_, t_, t_current);

  // printf("left_ (%.2f, %.2f, %.2f)", l_pose_.x, l_pose_.y, drive_imu_get());
  // printf("   right_ (%.2f, %.2f, %.2f)\n", r_pose_.x, r_pose_.y, drive_imu_get());

  // printf("left (%.2f, %.2f, %.2f)", l_pose.x, l_pose.y, drive_imu_get());
  // printf("   right (%.2f, %.2f, %.2f)", r_pose.x, r_pose.y, drive_imu_get());
  // printf("   current used (%.2f, %.2f, %.2f)      l delta: %.2f   r delta: %.2f\n", odom_current.x, odom_current.y, drive_imu_get(), l_, r_);
  // printf("        lo: %.2f  ro: %.2f  tw: %.2f\n", l_track_width, r_track_width, global_track_width);
}
