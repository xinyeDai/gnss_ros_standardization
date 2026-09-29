// SPDX-License-Identifier: MIT
#include <ros/ros.h>

#include <gnss_ros_standardization/GnssEphemerides.h>
#include <gnss_ros_standardization/GnssObservations.h>
#include <gnss_ros_standardization/ros1_gnss_core.hpp>

#include <algorithm>
#include <cctype>
#include <cstring>
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
    pnh_.param<bool>("use_gps_timestamp", use_gps_timestamp_, false);
    pnh_.param<int>("poll_period_ms", poll_period_ms_, 10);

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

    strinit(&stream_);
    if (!stropen(&stream_, stream_type_, STR_MODE_R, stream_path_.c_str())) {
      char stat[MAXSTRMSG] = {0};
      strstat(&stream_, stat);
      throw std::runtime_error(std::string("failed to open RTKLIB stream: ") + stat);
    }

    if (is_rtcm_) {
      if (!init_rtcm(&rtcm_)) throw std::runtime_error("init_rtcm failed");
      if (!receiver_option_.empty()) {
        std::strncpy(rtcm_.opt, receiver_option_.c_str(), sizeof(rtcm_.opt) - 1);
      }
      rtcm_initialized_ = true;
    } else {
      if (!init_raw(&raw_, format_)) throw std::runtime_error("init_raw failed");
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
        if (is_rtcm_) publishObservations(rtcm_.obs);
        else publishObservations(raw_.obs);
      } else if (ret == 2) {
        if (is_rtcm_) publishEphemerides(rtcm_.nav);
        else publishEphemerides(raw_.nav);
      } else if (ret < 0) {
        ROS_WARN_THROTTLE(2.0, "GNSS decoder reported an input error");
      }
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
  ros::Timer timer_;

  std::string format_name_;
  std::string stream_type_name_;
  std::string stream_path_;
  std::string receiver_option_;
  std::string frame_id_;
  std::string observation_topic_;
  std::string ephemeris_topic_;
  bool use_gps_timestamp_{false};
  int poll_period_ms_{10};

  int format_{STRFMT_UBX};
  int stream_type_{STR_SERIAL};
  bool is_rtcm_{false};
  bool raw_initialized_{false};
  bool rtcm_initialized_{false};

  stream_t stream_{};
  raw_t raw_{};
  rtcm_t rtcm_{};
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
