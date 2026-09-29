// SPDX-License-Identifier: MIT
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <gnss_ros_standardization/GnssEphemerides.h>
#include <gnss_ros_standardization/GnssObservations.h>
#include <gnss_ros_standardization/GnssSolution.h>
#include <gnss_ros_standardization/ros1_gnss_core.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

extern "C" {
#include "rtklib.h"
}

class RealTimeKinematicNode {
 public:
  RealTimeKinematicNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("rover_observation_topic", rover_topic_, "/rover/gnss/observation");
    pnh_.param<std::string>("base_observation_topic", base_topic_, "/base/gnss/observation");
    pnh_.param<std::string>("ephemeris_topic", eph_topic_, "/gnss/ephemeris");
    pnh_.param<std::string>("solution_topic", solution_topic_, "/gnss/solution");
    pnh_.param<std::string>("base_station_topic", base_station_topic_, "/base/gnss/station_ecef");

    pnh_.param<double>("match_tolerance_s", match_tolerance_s_, 0.002);
    pnh_.param<double>("max_base_age_s", max_base_age_s_, 30.0);
    pnh_.param<int>("base_queue_size", base_queue_size_, 100);
    pnh_.param<double>("elevation_mask_deg", elevation_mask_deg_, 15.0);

    pnh_.param<bool>("frequencies/l1", freq_.l1, true);
    pnh_.param<bool>("frequencies/l2", freq_.l2, true);
    pnh_.param<bool>("frequencies/l5", freq_.l5, false);

    pnh_.param<std::string>("base_position_mode", base_position_mode_, "rtcm");
    configureBasePosition();

    pnh_.param<bool>("auto_origin", auto_origin_, true);
    double olat = 0.0, olon = 0.0, oalt = 0.0;
    pnh_.param<double>("origin_latitude", olat, 0.0);
    pnh_.param<double>("origin_longitude", olon, 0.0);
    pnh_.param<double>("origin_altitude", oalt, 0.0);
    if (!auto_origin_) {
      const bool configured =
          std::fabs(olat) > 1e-12 || std::fabs(olon) > 1e-12 || std::fabs(oalt) > 1e-6;
      if (configured) {
        const double llh[3] = {olat * D2R, olon * D2R, oalt};
        pos2ecef(llh, origin_ecef_);
        origin_set_ = true;
      } else {
        ROS_WARN("Fixed ENU origin requested but left at zero; falling back to RTK base position.");
      }
    }

    std::memset(&nav_, 0, sizeof(nav_));
    initializeRtk();

    solution_pub_ = nh_.advertise<gnss_ros_standardization::GnssSolution>(
        solution_topic_, 20);
    eph_sub_ = nh_.subscribe(eph_topic_, 10, &RealTimeKinematicNode::onEphemerides, this);
    base_sub_ = nh_.subscribe(base_topic_, 200, &RealTimeKinematicNode::onBase, this);
    rover_sub_ = nh_.subscribe(rover_topic_, 200, &RealTimeKinematicNode::onRover, this);
    station_sub_ = nh_.subscribe(base_station_topic_, 5,
                                 &RealTimeKinematicNode::onBaseStation, this);

    ROS_INFO("RTK ready: rover=%s base=%s eph=%s solution=%s",
             rover_topic_.c_str(), base_topic_.c_str(),
             eph_topic_.c_str(), solution_topic_.c_str());
  }

  ~RealTimeKinematicNode() {
    rtkfree(&rtk_);
    std::lock_guard<std::mutex> lock(nav_mutex_);
    freenav(&nav_, 0xFF);
  }

 private:
  void configureBasePosition() {
    base_ecef_[0] = base_ecef_[1] = base_ecef_[2] = 0.0;
    if (base_position_mode_ == "llh") {
      double lat = 0.0, lon = 0.0, alt = 0.0;
      pnh_.param<double>("base_latitude", lat, 0.0);
      pnh_.param<double>("base_longitude", lon, 0.0);
      pnh_.param<double>("base_altitude", alt, 0.0);
      const double llh[3] = {lat * D2R, lon * D2R, alt};
      pos2ecef(llh, base_ecef_);
      base_position_set_ = norm(base_ecef_, 3) > 0.0;
    } else if (base_position_mode_ == "ecef") {
      pnh_.param<double>("base_x", base_ecef_[0], 0.0);
      pnh_.param<double>("base_y", base_ecef_[1], 0.0);
      pnh_.param<double>("base_z", base_ecef_[2], 0.0);
      base_position_set_ = norm(base_ecef_, 3) > 0.0;
    } else {
      base_position_mode_ = "rtcm";
    }
  }

  void initializeRtk() {
    prcopt_t opt = prcopt_default;
    opt.mode = PMODE_KINEMA;
    opt.elmin = elevation_mask_deg_ * D2R;
    opt.nf = freq_.l5 ? 3 : (freq_.l2 ? 2 : 1);

    int armode = ARMODE_CONT;
    double arthres = 3.0;
    int minfix = 10;
    int minfixsats = 4;
    int minholdsats = 5;
    double rejphase = 5.0;
    double rejcode = 30.0;
    pnh_.param<int>("ambiguity_mode", armode, ARMODE_CONT);
    pnh_.param<double>("ambiguity_threshold", arthres, 3.0);
    pnh_.param<int>("ambiguity_min_fix", minfix, 10);
    pnh_.param<int>("ambiguity_min_fix_sats", minfixsats, 4);
    pnh_.param<int>("ambiguity_min_hold_sats", minholdsats, 5);
    pnh_.param<double>("reject_phase_m", rejphase, 5.0);
    pnh_.param<double>("reject_code_m", rejcode, 30.0);

    opt.modear = armode;
    opt.thresar[0] = arthres;
    opt.minfix = minfix;
    opt.minfixsats = minfixsats;
    opt.minholdsats = minholdsats;
    opt.maxtdiff = max_base_age_s_;
    opt.maxinno[0] = rejphase;
    opt.maxinno[1] = rejcode;

    rtkinit(&rtk_, &opt);
    if (base_position_set_) {
      for (int i = 0; i < 3; ++i) rtk_.rb[i] = base_ecef_[i];
      ROS_INFO("RTK fixed base ECEF: %.3f %.3f %.3f",
               rtk_.rb[0], rtk_.rb[1], rtk_.rb[2]);
    } else {
      ROS_WARN("RTK base position waiting for RTCM station ECEF");
    }
  }

  static double continuousTow(const gnss_ros_standardization::GnssObservations& m) {
    return static_cast<double>(m.week) * 604800.0 + m.tow;
  }

  void onBaseStation(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (base_position_mode_ != "rtcm") return;
    std::lock_guard<std::mutex> lock(obs_mutex_);
    base_ecef_[0] = msg->point.x;
    base_ecef_[1] = msg->point.y;
    base_ecef_[2] = msg->point.z;
    if (norm(base_ecef_, 3) <= 0.0) return;
    for (int i = 0; i < 3; ++i) rtk_.rb[i] = base_ecef_[i];
    if (!base_position_set_) {
      ROS_INFO("RTK base position received from RTCM: %.3f %.3f %.3f",
               rtk_.rb[0], rtk_.rb[1], rtk_.rb[2]);
    }
    base_position_set_ = true;
  }

  void onEphemerides(const gnss_ros_standardization::GnssEphemerides::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    gnss_ros1::ingestEphemerides(nav_, *msg);
  }

  void onBase(const gnss_ros_standardization::GnssObservations::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(obs_mutex_);
    base_queue_.push_back(msg);
    while (static_cast<int>(base_queue_.size()) > base_queue_size_) base_queue_.pop_front();
  }

  gnss_ros_standardization::GnssObservations::ConstPtr findBase(
      const gnss_ros_standardization::GnssObservations& rover) {
    const double tr = continuousTow(rover);
    gnss_ros_standardization::GnssObservations::ConstPtr exact;
    gnss_ros_standardization::GnssObservations::ConstPtr aged;
    double best_abs = std::numeric_limits<double>::infinity();
    double best_age = std::numeric_limits<double>::infinity();

    for (std::deque<gnss_ros_standardization::GnssObservations::ConstPtr>::const_iterator
             it = base_queue_.begin(); it != base_queue_.end(); ++it) {
      const double tb = continuousTow(**it);
      const double dt = tr - tb;
      if (std::fabs(dt) <= match_tolerance_s_ && std::fabs(dt) < best_abs) {
        exact = *it;
        best_abs = std::fabs(dt);
      }
      if (dt >= -match_tolerance_s_ && dt <= max_base_age_s_ && dt < best_age) {
        aged = *it;
        best_age = dt;
      }
    }
    return exact ? exact : aged;
  }

  void onRover(const gnss_ros_standardization::GnssObservations::ConstPtr& rover) {
    gnss_ros_standardization::GnssObservations::ConstPtr base;
    {
      std::lock_guard<std::mutex> lock(obs_mutex_);
      if (!base_position_set_) {
        ROS_WARN_THROTTLE(2.0, "RTK: base position not available");
        return;
      }
      base = findBase(*rover);
    }
    if (!base) {
      ROS_WARN_THROTTLE(2.0, "RTK: no base observation within %.2f s", max_base_age_s_);
      return;
    }

    std::lock_guard<std::mutex> nav_lock(nav_mutex_);
    if (nav_.n == 0 && nav_.ng == 0) {
      ROS_WARN_THROTTLE(5.0, "RTK: waiting for ephemerides");
      return;
    }

    std::vector<obsd_t> obs = gnss_ros1::buildObsEpoch(*rover, base.get(), freq_);
    if (obs.empty()) return;

    const int ok = rtkpos(&rtk_, obs.data(), static_cast<int>(obs.size()), &nav_);
    if (ok <= 0 || rtk_.sol.stat == SOLQ_NONE) {
      ROS_WARN_THROTTLE(2.0, "RTKLIB rtkpos did not produce a solution");
      return;
    }

    if (!origin_set_) {
      if (norm(base_ecef_, 3) > 0.0) {
        for (int i = 0; i < 3; ++i) origin_ecef_[i] = base_ecef_[i];
      } else {
        origin_ecef_[0] = rtk_.sol.rr[0];
        origin_ecef_[1] = rtk_.sol.rr[1];
        origin_ecef_[2] = rtk_.sol.rr[2];
      }
      origin_set_ = true;
    }

    gnss_ros_standardization::GnssSolution out =
        gnss_ros1::makeSolution(rtk_.sol, rtk_.ssat, rtk_.opt.elmin,
                                rover->header.stamp, origin_ecef_);
    solution_pub_.publish(out);

    ROS_INFO_THROTTLE(1.0,
        "RTK: status=%u lat=%.8f lon=%.8f h=%.3f ratio=%.2f sats=%u age=%.2f",
        out.status, out.latitude, out.longitude, out.altitude,
        out.ratio, out.num_sats, out.age_diff);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber rover_sub_, base_sub_, eph_sub_, station_sub_;
  ros::Publisher solution_pub_;

  std::string rover_topic_, base_topic_, eph_topic_, solution_topic_, base_station_topic_;
  std::string base_position_mode_;
  double match_tolerance_s_{0.002};
  double max_base_age_s_{30.0};
  int base_queue_size_{100};
  double elevation_mask_deg_{15.0};

  gnss_ros1::FrequencyMask freq_;
  std::deque<gnss_ros_standardization::GnssObservations::ConstPtr> base_queue_;
  std::mutex obs_mutex_;
  std::mutex nav_mutex_;

  nav_t nav_{};
  rtk_t rtk_{};
  double base_ecef_[3]{0.0, 0.0, 0.0};
  bool base_position_set_{false};

  bool auto_origin_{false};
  bool origin_set_{false};
  double origin_ecef_[3]{0.0, 0.0, 0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "real_time_kinematic");
  RealTimeKinematicNode node;
  ros::spin();
  return 0;
}
