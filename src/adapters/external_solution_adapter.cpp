#include <ros/ros.h>

#include <gnss_ros_standardization/ExternalGnssSolution.h>
#include <gnss_ros_standardization/GnssSolution.h>

#include <Eigen/Core>
#include <boost/array.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace grs = gnss_ros_standardization;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

Eigen::Vector3d llhToEcef(double lat_deg, double lon_deg, double h) {
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double s = std::sin(lat);
  const double c = std::cos(lat);
  const double sl = std::sin(lon);
  const double cl = std::cos(lon);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * s * s);

  return Eigen::Vector3d(
      (n + h) * c * cl,
      (n + h) * c * sl,
      (n * (1.0 - kWgs84E2) + h) * s);
}

// Rotation from ECEF vector to local ENU vector at (lat, lon).
Eigen::Matrix3d ecefToEnuRotation(double lat_deg, double lon_deg) {
  const double lat = lat_deg * kDegToRad;
  const double lon = lon_deg * kDegToRad;
  const double sphi = std::sin(lat);
  const double cphi = std::cos(lat);
  const double slam = std::sin(lon);
  const double clam = std::cos(lon);

  Eigen::Matrix3d r;
  r << -slam,          clam,         0.0,
       -sphi * clam,  -sphi * slam,  cphi,
        cphi * clam,   cphi * slam,  sphi;
  return r;
}

Eigen::Matrix3d arrayToMatrix(const boost::array<double, 9>& a) {
  Eigen::Matrix3d m;
  m << a[0], a[1], a[2],
       a[3], a[4], a[5],
       a[6], a[7], a[8];
  return m;
}

void matrixToArray(const Eigen::Matrix3d& m, boost::array<double, 9>& a) {
  a[0] = m(0, 0); a[1] = m(0, 1); a[2] = m(0, 2);
  a[3] = m(1, 0); a[4] = m(1, 1); a[5] = m(1, 2);
  a[6] = m(2, 0); a[7] = m(2, 1); a[8] = m(2, 2);
}

bool finiteLlh(double lat, double lon, double h) {
  return std::isfinite(lat) && std::isfinite(lon) && std::isfinite(h) &&
         lat >= -90.0 && lat <= 90.0 &&
         lon >= -180.0 && lon <= 180.0;
}

}  // namespace

class ExternalSolutionAdapter {
 public:
  ExternalSolutionAdapter() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/gnss/external_solution");
    pnh_.param<std::string>("output_topic", output_topic_, "/gnss/solution");
    pnh_.param<std::string>("frame_id", frame_id_, "gnss_link");
    pnh_.param<bool>("auto_origin", auto_origin_, true);

    double lat = 0.0, lon = 0.0, alt = 0.0;
    pnh_.param<double>("origin_latitude", lat, 0.0);
    pnh_.param<double>("origin_longitude", lon, 0.0);
    pnh_.param<double>("origin_altitude", alt, 0.0);

    if (!auto_origin_) {
      const bool configured =
          std::fabs(lat) > 1e-12 || std::fabs(lon) > 1e-12 || std::fabs(alt) > 1e-6;
      if (!configured || !finiteLlh(lat, lon, alt)) {
        ROS_FATAL("Fixed ENU origin requested but no valid non-zero BLH was configured.");
        throw std::runtime_error("invalid fixed ENU origin");
      }
      setOrigin(lat, lon, alt);
      ROS_INFO("Using fixed ENU origin: lat=%.9f lon=%.9f alt=%.3f", lat, lon, alt);
    } else {
      ROS_INFO("ENU origin will be initialized from the first valid external solution.");
    }

    pub_ = nh_.advertise<grs::GnssSolution>(output_topic_, 20);
    sub_ = nh_.subscribe(input_topic_, 100, &ExternalSolutionAdapter::callback, this);

    ROS_INFO("External GNSS solution adapter: %s -> %s",
             input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  void setOrigin(double lat_deg, double lon_deg, double alt) {
    origin_lat_deg_ = lat_deg;
    origin_lon_deg_ = lon_deg;
    origin_alt_ = alt;
    origin_ecef_ = llhToEcef(lat_deg, lon_deg, alt);
    r_ecef_to_enu_origin_ = ecefToEnuRotation(lat_deg, lon_deg);
    origin_set_ = true;
  }

  void callback(const grs::ExternalGnssSolution::ConstPtr& in) {
    if (in->header.stamp.isZero()) {
      ROS_WARN_THROTTLE(5.0,
          "External GNSS header.stamp is zero; downstream time synchronization may fail.");
    }
    if (in->status > grs::ExternalGnssSolution::STATUS_EKF) {
      ROS_WARN_THROTTLE(2.0, "Rejecting external GNSS solution with unknown status=%u.",
                        static_cast<unsigned>(in->status));
      return;
    }
    if (in->covariance_frame != grs::ExternalGnssSolution::COVARIANCE_FRAME_ENU &&
        in->covariance_frame != grs::ExternalGnssSolution::COVARIANCE_FRAME_ECEF) {
      ROS_WARN_THROTTLE(2.0, "Rejecting external GNSS solution with invalid covariance_frame.");
      return;
    }
    if (in->has_velocity &&
        in->velocity_frame != grs::ExternalGnssSolution::VELOCITY_FRAME_ENU &&
        in->velocity_frame != grs::ExternalGnssSolution::VELOCITY_FRAME_ECEF) {
      ROS_WARN_THROTTLE(2.0, "Rejecting external GNSS solution with invalid velocity_frame.");
      return;
    }
    if (in->has_gnss_time &&
        (in->time_week == 0 || !std::isfinite(in->time_tow) ||
         in->time_tow < 0.0 || in->time_tow >= 604800.0)) {
      ROS_WARN_THROTTLE(2.0, "Rejecting external GNSS solution with invalid GPS week/TOW.");
      return;
    }
    if (!finiteLlh(in->latitude, in->longitude, in->altitude)) {
      ROS_WARN_THROTTLE(2.0, "Rejecting external GNSS solution with invalid BLH.");
      return;
    }

    if (!origin_set_) {
      setOrigin(in->latitude, in->longitude, in->altitude);
      ROS_INFO("Auto ENU origin initialized: lat=%.9f lon=%.9f alt=%.3f",
               in->latitude, in->longitude, in->altitude);
    }

    grs::GnssSolution out;
    out.header = in->header;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = frame_id_;
    }

    if (in->has_gnss_time) {
      out.time_week = in->time_week;
      out.time_tow = in->time_tow;
    } else {
      out.time_week = 0;
      out.time_tow = std::numeric_limits<double>::quiet_NaN();
    }

    out.solution_source = grs::GnssSolution::SOLUTION_SOURCE_EXTERNAL;
    out.status = in->status;
    out.num_sats = in->num_sats;
    out.ratio = in->ratio;
    out.age_diff = in->age_diff;
    out.gdop = in->gdop;
    out.pdop = in->pdop;
    out.hdop = in->hdop;
    out.vdop = in->vdop;

    out.latitude = in->latitude;
    out.longitude = in->longitude;
    out.altitude = in->altitude;

    const Eigen::Vector3d p_ecef =
        llhToEcef(in->latitude, in->longitude, in->altitude);
    out.pos_ecef.x = p_ecef.x();
    out.pos_ecef.y = p_ecef.y();
    out.pos_ecef.z = p_ecef.z();

    out.pos_enu_org_ecef.x = origin_ecef_.x();
    out.pos_enu_org_ecef.y = origin_ecef_.y();
    out.pos_enu_org_ecef.z = origin_ecef_.z();

    const Eigen::Vector3d p_enu =
        r_ecef_to_enu_origin_ * (p_ecef - origin_ecef_);
    out.pos_enu.x = p_enu.x();
    out.pos_enu.y = p_enu.y();
    out.pos_enu.z = p_enu.z();

    const Eigen::Matrix3d r_cur = ecefToEnuRotation(in->latitude, in->longitude);
    const Eigen::Matrix3d cov_in = arrayToMatrix(in->position_covariance);
    Eigen::Matrix3d cov_ecef;
    Eigen::Matrix3d cov_enu;

    if (in->covariance_frame ==
        grs::ExternalGnssSolution::COVARIANCE_FRAME_ECEF) {
      cov_ecef = cov_in;
      cov_enu = r_cur * cov_ecef * r_cur.transpose();
    } else {
      cov_enu = cov_in;
      cov_ecef = r_cur.transpose() * cov_enu * r_cur;
    }
    matrixToArray(cov_ecef, out.pos_cov_ecef);
    matrixToArray(cov_enu, out.pos_enu_cov);

    if (in->has_velocity) {
      const Eigen::Vector3d v_in(in->velocity.x, in->velocity.y, in->velocity.z);
      const Eigen::Matrix3d v_cov_in = arrayToMatrix(in->velocity_covariance);
      Eigen::Vector3d v_ecef;
      Eigen::Vector3d v_enu;
      Eigen::Matrix3d v_cov_ecef;
      Eigen::Matrix3d v_cov_enu;

      if (in->velocity_frame ==
          grs::ExternalGnssSolution::VELOCITY_FRAME_ECEF) {
        v_ecef = v_in;
        v_enu = r_cur * v_ecef;
        v_cov_ecef = v_cov_in;
        v_cov_enu = r_cur * v_cov_ecef * r_cur.transpose();
      } else {
        v_enu = v_in;
        v_ecef = r_cur.transpose() * v_enu;
        v_cov_enu = v_cov_in;
        v_cov_ecef = r_cur.transpose() * v_cov_enu * r_cur;
      }

      out.vel_ecef.x = v_ecef.x();
      out.vel_ecef.y = v_ecef.y();
      out.vel_ecef.z = v_ecef.z();
      out.vel_enu.x = v_enu.x();
      out.vel_enu.y = v_enu.y();
      out.vel_enu.z = v_enu.z();
      matrixToArray(v_cov_ecef, out.vel_cov_ecef);
      matrixToArray(v_cov_enu, out.vel_enu_cov);
    } else {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      out.vel_ecef.x = out.vel_ecef.y = out.vel_ecef.z = nan;
      out.vel_enu.x = out.vel_enu.y = out.vel_enu.z = nan;
      std::fill(out.vel_cov_ecef.begin(), out.vel_cov_ecef.end(), nan);
      std::fill(out.vel_enu_cov.begin(), out.vel_enu_cov.end(), nan);
    }

    pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  bool auto_origin_{false};
  bool origin_set_{false};
  double origin_lat_deg_{0.0};
  double origin_lon_deg_{0.0};
  double origin_alt_{0.0};
  Eigen::Vector3d origin_ecef_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d r_ecef_to_enu_origin_{Eigen::Matrix3d::Identity()};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "external_solution_adapter");
  try {
    ExternalSolutionAdapter node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("external_solution_adapter failed: %s", e.what());
    return 1;
  }
  return 0;
}
