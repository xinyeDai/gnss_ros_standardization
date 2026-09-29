// SPDX-License-Identifier: MIT
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <gnss_ros_standardization/GnssEphemerides.h>
#include <gnss_ros_standardization/GnssObservations.h>
#include <gnss_ros_standardization/ros1_gnss_core.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include "rtklib.h"
}

class RawGnssDecoderNode {
 public:
  RawGnssDecoderNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("format", format_name_, "ubx");
    pnh_.param<std::string>("stream_type", stream_type_name_, "serial");
    pnh_.param<std::string>("stream_path", stream_path_, "");
    pnh_.param<std::string>("receiver_option", receiver_option_, "");
    pnh_.param<std::string>("frame_id", frame_id_, "gnss_receiver");
    pnh_.param<std::string>("observation_topic", observation_topic_, "/gnss/observation");
    pnh_.param<std::string>("ephemeris_topic", ephemeris_topic_, "/gnss/ephemeris");
    pnh_.param<std::string>("station_topic", station_topic_, "/gnss/station_ecef");
    pnh_.param<bool>("use_gps_timestamp", use_gps_timestamp_, false);
    pnh_.param<int>("poll_period_ms", poll_period_ms_, 10);
    pnh_.param<int>("rtcm_epoch_hold_ms", rtcm_epoch_hold_ms_, 20);

    if (stream_path_.empty()) {
      throw std::runtime_error("~stream_path is required");
    }
    if (poll_period_ms_ < 1) poll_period_ms_ = 1;

    format_ = parseFormat(format_name_);
    stream_type_ = parseStreamType(stream_type_name_);
    is_rtcm_ = (format_ == STRFMT_RTCM3);

    obs_pub_ = nh_.advertise<gnss_ros_standardization::GnssObservations>(
        observation_topic_, 20);
    eph_pub_ = nh_.advertise<gnss_ros_standardization::GnssEphemerides>(
        ephemeris_topic_, 1, true);
    station_pub_ = nh_.advertise<geometry_msgs::PointStamped>(station_topic_, 1, true);

    strinit(&stream_);
    if (!stropen(&stream_, stream_type_, STR_MODE_R, stream_path_.c_str())) {
      char stat[MAXSTRMSG] = {0};
      strstat(&stream_, stat);
      throw std::runtime_error(std::string("failed to open RTKLIB stream: ") + stat);
    }

    const ros::Time now_ros = ros::Time::now();
    gtime_t now_utc{};
    now_utc.time = static_cast<time_t>(now_ros.sec);
    now_utc.sec = static_cast<double>(now_ros.nsec) * 1e-9;
    const gtime_t now_gpst = utc2gpst(now_utc);

    if (is_rtcm_) {
      if (!init_rtcm(&rtcm_)) throw std::runtime_error("init_rtcm failed");
      // RTCM messages often carry only time-of-week/day. RTKLIB needs an
      // approximate absolute epoch to resolve week/day rollover correctly.
      rtcm_.time = now_gpst;
      if (!receiver_option_.empty()) {
        std::strncpy(rtcm_.opt, receiver_option_.c_str(), sizeof(rtcm_.opt) - 1);
      }
      rtcm_initialized_ = true;
    } else {
      if (!init_raw(&raw_, format_)) throw std::runtime_error("init_raw failed");
      // Also seed raw receiver time; formats with incomplete date/week fields
      // can use this as the same rollover reference.
      raw_.time = now_gpst;
      if (!receiver_option_.empty()) {
        std::strncpy(raw_.opt, receiver_option_.c_str(), sizeof(raw_.opt) - 1);
      }
      raw_initialized_ = true;
    }

    timer_ = nh_.createTimer(
        ros::Duration(static_cast<double>(poll_period_ms_) / 1000.0),
        &RawGnssDecoderNode::onTimer, this);

    ROS_INFO("Raw GNSS decoder started: format=%s stream_type=%s stream_path=%s",
             format_name_.c_str(), stream_type_name_.c_str(), stream_path_.c_str());
  }

  ~RawGnssDecoderNode() {
    strclose(&stream_);
    if (raw_initialized_) free_raw(&raw_);
    if (rtcm_initialized_) free_rtcm(&rtcm_);
  }

 private:
  static std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
  }

  static int parseFormat(const std::string& name) {
    const std::string s = lower(name);
    if (s == "ubx" || s == "ublox") return STRFMT_UBX;
    if (s == "sbf" || s == "septentrio" || s == "sept") return STRFMT_SEPT;
    if (s == "novatel" || s == "oem4" || s == "oem6" || s == "oem7") return STRFMT_OEM4;
#ifdef STRFMT_OEM3
    if (s == "oem3") return STRFMT_OEM3;
#endif
    if (s == "rtcm3" || s == "rtcm") return STRFMT_RTCM3;
    throw std::runtime_error("unsupported ~format: " + name);
  }

  static int parseStreamType(const std::string& name) {
    const std::string s = lower(name);
    if (s == "serial") return STR_SERIAL;
    if (s == "file") return STR_FILE;
    if (s == "tcp" || s == "tcp_client") return STR_TCPCLI;
    if (s == "ntrip" || s == "ntrip_client") return STR_NTRIPCLI;
#ifdef STR_UDPCLI
    if (s == "udp" || s == "udp_client") return STR_UDPCLI;
#endif
    throw std::runtime_error("unsupported ~stream_type: " + name);
  }

  void onTimer(const ros::TimerEvent&) {
    unsigned char buf[8192];
    const int n = strread(&stream_, buf, sizeof(buf));
    if (n < 0) {
      ROS_WARN_THROTTLE(2.0, "GNSS stream read error");
      return;
    }
    for (int i = 0; i < n; ++i) {
      const int ret = is_rtcm_ ? input_rtcm3(&rtcm_, buf[i])
                               : input_raw(&raw_, format_, buf[i]);
      if (ret == 1) {
        if (is_rtcm_) accumulateRtcmObservations(rtcm_.obs);
        else publishObservations(raw_.obs);
      } else if (ret == 2) {
        if (is_rtcm_) publishEphemerides(rtcm_.nav);
        else publishEphemerides(raw_.nav);
      } else if (is_rtcm_ && ret == 5) {
        publishRtcmStation();
      } else if (ret < 0) {
        ROS_WARN_THROTTLE(2.0, "GNSS decoder reported an input error");
      }
    }
    if (is_rtcm_) flushRtcmEpochs();
  }

  struct RtcmEpochBuffer {
    int week{0};
    double tow{0.0};
    ros::WallTime first_seen;
    std::vector<gnss_ros_standardization::GnssObservation> observations;
  };

  void accumulateRtcmObservations(const obs_t& obs) {
    if (obs.n <= 0 || obs.data == NULL) return;
    int week = 0;
    const gtime_t epoch = obs.data[0].time;
    const double tow = time2gpst(epoch, &week);
    if (week <= 0 || !std::isfinite(tow)) return;

    const long long key =
        static_cast<long long>(week) * 604800000LL +
        static_cast<long long>(std::llround(tow * 1000.0));
    RtcmEpochBuffer& dst = rtcm_epochs_[key];
    if (dst.first_seen.isZero()) {
      dst.week = week;
      dst.tow = tow;
      dst.first_seen = ros::WallTime::now();
    }

    for (int i = 0; i < obs.n; ++i) {
      for (int k = 0; k < NFREQ + NEXOBS; ++k) {
        const bool empty = obs.data[i].P[k] == 0.0 &&
                           obs.data[i].L[k] == 0.0 &&
                           obs.data[i].D[k] == 0.0 &&
                           obs.data[i].SNR[k] == 0;
        if (empty) continue;
        const gnss_ros_standardization::GnssObservation m =
            gnss_ros1::obsToMsg(obs.data[i], k);

        bool replaced = false;
        for (size_t j = 0; j < dst.observations.size(); ++j) {
          if (dst.observations[j].sat == m.sat &&
              dst.observations[j].code == m.code) {
            dst.observations[j] = m;
            replaced = true;
            break;
          }
        }
        if (!replaced) dst.observations.push_back(m);
      }
    }
  }

  void flushRtcmEpochs() {
    if (rtcm_epochs_.empty()) return;
    const ros::Time stamp_now = ros::Time::now();
    const ros::WallTime wall_now = ros::WallTime::now();

    for (std::map<long long, RtcmEpochBuffer>::iterator it = rtcm_epochs_.begin();
         it != rtcm_epochs_.end();) {
      const bool has_newer = std::next(it) != rtcm_epochs_.end();
      const double age_ms = (wall_now - it->second.first_seen).toSec() * 1000.0;
      if (!has_newer && age_ms < static_cast<double>(rtcm_epoch_hold_ms_)) {
        ++it;
        continue;
      }

      gnss_ros_standardization::GnssObservations msg;
      const gtime_t gpst =
          gpst2time(it->second.week, it->second.tow);
      msg.header.stamp =
          use_gps_timestamp_ ? gnss_ros1::gpstToUtcRosTime(gpst) : stamp_now;
      msg.header.frame_id = frame_id_;
      msg.week = static_cast<uint32_t>(it->second.week);
      msg.tow = it->second.tow;
      msg.observations.swap(it->second.observations);
      if (!msg.observations.empty()) {
        obs_pub_.publish(msg);
        ROS_INFO_THROTTLE(1.0,
            "RTCM GNSS: week=%u tow=%.3f aggregated_signals=%zu",
            msg.week, msg.tow, msg.observations.size());
      }
      it = rtcm_epochs_.erase(it);
    }
  }

  void publishObservations(const obs_t& obs) {
    if (obs.n <= 0 || obs.data == NULL) return;

    int week = 0;
    const gtime_t epoch = obs.data[0].time;
    const double tow = time2gpst(epoch, &week);
    if (week <= 0 || !std::isfinite(tow)) return;

    gnss_ros_standardization::GnssObservations msg;
    msg.header.stamp = use_gps_timestamp_ ? gnss_ros1::gpstToUtcRosTime(epoch)
                                          : ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint32_t>(week);
    msg.tow = tow;

    for (int i = 0; i < obs.n; ++i) {
      for (int k = 0; k < NFREQ + NEXOBS; ++k) {
        const bool empty = obs.data[i].P[k] == 0.0 &&
                           obs.data[i].L[k] == 0.0 &&
                           obs.data[i].D[k] == 0.0 &&
                           obs.data[i].SNR[k] == 0;
        if (!empty) msg.observations.push_back(gnss_ros1::obsToMsg(obs.data[i], k));
      }
    }

    if (!msg.observations.empty()) {
      obs_pub_.publish(msg);
      ROS_INFO_THROTTLE(1.0, "Raw GNSS: week=%u tow=%.3f signals=%zu",
                        msg.week, msg.tow, msg.observations.size());
    }
  }


  void publishRtcmStation() {
    if (norm(rtcm_.sta.pos, 3) <= 0.0) return;
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "ecef";
    msg.point.x = rtcm_.sta.pos[0];
    msg.point.y = rtcm_.sta.pos[1];
    msg.point.z = rtcm_.sta.pos[2];
    station_pub_.publish(msg);
    ROS_INFO_THROTTLE(10.0, "RTCM station ECEF: %.3f %.3f %.3f",
                      msg.point.x, msg.point.y, msg.point.z);
  }

  void publishEphemerides(const nav_t& nav) {
    gnss_ros_standardization::GnssEphemerides msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;

    for (int i = 0; i < nav.n; ++i) {
      if (nav.eph[i].sat > 0) msg.gnss_ephemeris.push_back(gnss_ros1::ephToMsg(nav.eph[i]));
    }
    for (int i = 0; i < nav.ng; ++i) {
      if (nav.geph[i].sat > 0) msg.glonass_ephemeris.push_back(gnss_ros1::gephToMsg(nav.geph[i]));
    }

    if (!msg.gnss_ephemeris.empty() || !msg.glonass_ephemeris.empty()) {
      eph_pub_.publish(msg);
      ROS_INFO_THROTTLE(5.0, "Raw GNSS ephemerides: GNSS=%zu GLO=%zu",
                        msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher obs_pub_;
  ros::Publisher eph_pub_;
  ros::Publisher station_pub_;
  ros::Timer timer_;

  std::string format_name_;
  std::string stream_type_name_;
  std::string stream_path_;
  std::string receiver_option_;
  std::string frame_id_;
  std::string observation_topic_;
  std::string ephemeris_topic_;
  std::string station_topic_;
  bool use_gps_timestamp_{false};
  int poll_period_ms_{10};
  int rtcm_epoch_hold_ms_{20};

  int format_{STRFMT_UBX};
  int stream_type_{STR_SERIAL};
  bool is_rtcm_{false};
  bool raw_initialized_{false};
  bool rtcm_initialized_{false};

  stream_t stream_{};
  raw_t raw_{};
  rtcm_t rtcm_{};
  std::map<long long, RtcmEpochBuffer> rtcm_epochs_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "raw_gnss_decoder");
  try {
    RawGnssDecoderNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("raw_gnss_decoder: %s", e.what());
    return 1;
  }
  return 0;
}
