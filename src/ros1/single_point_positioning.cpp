// SPDX-License-Identifier: MIT
#include <ros/ros.h>

#include <gnss_ros_standardization/GnssEphemerides.h>
#include <gnss_ros_standardization/GnssObservations.h>
#include <gnss_ros_standardization/GnssSolution.h>
#include <gnss_ros_standardization/ros1_gnss_core.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

extern "C" {
#include "rtklib.h"
}

class SinglePointPositioningNode {
 public:
  SinglePointPositioningNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("observation_topic", observation_topic_, "/gnss/observation");
    pnh_.param<std::string>("ephemeris_topic", ephemeris_topic_, "/gnss/ephemeris");
    pnh_.param<std::string>("solution_topic", solution_topic_, "/gnss/solution");
    pnh_.param<double>("elevation_mask_deg", elevation_mask_deg_, 15.0);
    pnh_.param<bool>("frequencies/l1", freq_.l1, true);
    pnh_.param<bool>("frequencies/l2", freq_.l2, true);
    pnh_.param<bool>("frequencies/l5", freq_.l5, true);
    pnh_.param<bool>("auto_origin", auto_origin_, true);

    double lat = 0.0, lon = 0.0, alt = 0.0;
    pnh_.param<double>("origin_latitude", lat, 0.0);
    pnh_.param<double>("origin_longitude", lon, 0.0);
    pnh_.param<double>("origin_altitude", alt, 0.0);
    if (!auto_origin_) {
      const bool configured =
          std::fabs(lat) > 1e-12 || std::fabs(lon) > 1e-12 || std::fabs(alt) > 1e-6;
      if (configured) {
        const double llh[3] = {lat * D2R, lon * D2R, alt};
        pos2ecef(llh, origin_ecef_);
        origin_set_ = true;
      } else {
        ROS_WARN("Fixed ENU origin requested but left at zero; using first SPP solution.");
      }
    }

    std::memset(&nav_, 0, sizeof(nav_));
    opt_ = prcopt_default;
    opt_.mode = PMODE_SINGLE;
    opt_.elmin = elevation_mask_deg_ * D2R;
    opt_.ionoopt = IONOOPT_BRDC;
    opt_.tropopt = TROPOPT_SAAS;
    opt_.sateph = EPHOPT_BRDC;
    opt_.nf = freq_.l5 ? 3 : (freq_.l2 ? 2 : 1);

    solution_pub_ = nh_.advertise<gnss_ros_standardization::GnssSolution>(
        solution_topic_, 20);
    eph_sub_ = nh_.subscribe(ephemeris_topic_, 10,
                             &SinglePointPositioningNode::onEphemerides, this);
    obs_sub_ = nh_.subscribe(observation_topic_, 100,
                             &SinglePointPositioningNode::onObservations, this);

    ROS_INFO("SPP ready: obs=%s eph=%s solution=%s",
             observation_topic_.c_str(), ephemeris_topic_.c_str(), solution_topic_.c_str());
  }

  ~SinglePointPositioningNode() {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    freenav(&nav_, 0xFF);
  }

 private:
  void onEphemerides(const gnss_ros_standardization::GnssEphemerides::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    gnss_ros1::ingestEphemerides(nav_, *msg);
  }

  void onObservations(const gnss_ros_standardization::GnssObservations::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    if (nav_.n == 0 && nav_.ng == 0) {
      ROS_WARN_THROTTLE(5.0, "SPP: waiting for ephemerides");
      return;
    }

    std::vector<obsd_t> obs = gnss_ros1::buildObsEpoch(*msg, NULL, freq_);
    if (obs.size() < 4) {
      ROS_WARN_THROTTLE(2.0, "SPP: fewer than 4 usable satellites");
      return;
    }

    sol_t sol{};
    ssat_t ssat[MAXSAT]{};
    double azel[MAXSAT * 2] = {0};
    char err[1024] = {0};

    const int ok = pntpos(obs.data(), static_cast<int>(obs.size()), &nav_, &opt_,
                          &sol, azel, ssat, err);
    if (!ok) {
      ROS_WARN_THROTTLE(2.0, "SPP failed: %s", err[0] ? err : "pntpos returned 0");
      return;
    }

    if (!origin_set_) {
      origin_ecef_[0] = sol.rr[0];
      origin_ecef_[1] = sol.rr[1];
      origin_ecef_[2] = sol.rr[2];
      origin_set_ = true;
      ROS_INFO("SPP ENU origin initialized from first solution");
    }

    gnss_ros_standardization::GnssSolution out =
        gnss_ros1::makeSolution(sol, ssat, opt_.elmin, msg->header.stamp, origin_ecef_);
    solution_pub_.publish(out);

    ROS_INFO_THROTTLE(1.0,
        "SPP: status=%u lat=%.8f lon=%.8f h=%.3f sats=%u",
        out.status, out.latitude, out.longitude, out.altitude, out.num_sats);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber obs_sub_;
  ros::Subscriber eph_sub_;
  ros::Publisher solution_pub_;

  std::string observation_topic_;
  std::string ephemeris_topic_;
  std::string solution_topic_;
  double elevation_mask_deg_{15.0};
  bool auto_origin_{true};
  bool origin_set_{false};
  double origin_ecef_[3]{0.0, 0.0, 0.0};

  gnss_ros1::FrequencyMask freq_;
  prcopt_t opt_{};
  nav_t nav_{};
  std::mutex nav_mutex_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "single_point_positioning");
  SinglePointPositioningNode node;
  ros::spin();
  return 0;
}
