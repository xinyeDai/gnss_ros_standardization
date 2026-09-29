std_msgs/Header header  # stamp = ROS measurement time; frame_id = receiver/source label.
                        # For canonical GNSS time, use time_week / time_tow when available.

# GNSS Time (GPS Time System)
# External solutions may not provide GPST explicitly. In that case:
#   time_week = 0
#   time_tow  = NaN
uint32 time_week
float64 time_tow

# Solution Origin
uint8 solution_source
uint8 SOLUTION_SOURCE_UNKNOWN=0
uint8 SOLUTION_SOURCE_BINARY=1
uint8 SOLUTION_SOURCE_NMEA=2
uint8 SOLUTION_SOURCE_COMPUTED=3
uint8 SOLUTION_SOURCE_EXTERNAL=4  # already-solved PVT/RTK/INS output normalized by an adapter

# Solution Status
uint8 status
uint8 STATUS_NONE=0
uint8 STATUS_FIX=1
uint8 STATUS_FLOAT=2
uint8 STATUS_SBAS=3
uint8 STATUS_DGPS=4
uint8 STATUS_SINGLE=5
uint8 STATUS_PPP=6
uint8 STATUS_EKF=7

uint8 num_sats
float32 ratio
float32 age_diff
float32 gdop
float32 pdop
float32 hdop
float32 vdop

# Global Position — WGS84 ellipsoidal coordinates
float64 latitude
float64 longitude
float64 altitude

# Global Position — Earth-Centered Earth-Fixed (ECEF, WGS84)
geometry_msgs/Point pos_ecef
float64[9] pos_cov_ecef

# Global Velocity — ECEF
geometry_msgs/Vector3 vel_ecef
float64[9] vel_cov_ecef

# Local Coordinate System Origin — ECEF
geometry_msgs/Point pos_enu_org_ecef

# Local Position — East-North-Up (ENU) relative to pos_enu_org_ecef
geometry_msgs/Point pos_enu

# Position covariance in the local ENU tangent frame at the current solution
float64[9] pos_enu_cov

# Local Velocity — ENU
geometry_msgs/Vector3 vel_enu

# Velocity covariance in the local ENU tangent frame
float64[9] vel_enu_cov
