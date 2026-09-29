# Minimal standardized input for already-solved GNSS/PVT/RTK data.
# Use this when an upstream receiver/INS/driver already provides BLH + covariance + status.
std_msgs/Header header

# Optional canonical GPST. When unavailable set has_gnss_time=false.
bool has_gnss_time
uint32 time_week
float64 time_tow

# Standardized solution status.
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

# WGS84 geodetic position. altitude is ellipsoidal height [m].
float64 latitude
float64 longitude
float64 altitude

# Position covariance [m^2], row-major 3x3.
# ENU means East/North/Up tangent axes at the current receiver position.
uint8 covariance_frame
uint8 COVARIANCE_FRAME_ENU=0
uint8 COVARIANCE_FRAME_ECEF=1
float64[9] position_covariance

# Optional velocity and covariance.
bool has_velocity
uint8 velocity_frame
uint8 VELOCITY_FRAME_ENU=0
uint8 VELOCITY_FRAME_ECEF=1
geometry_msgs/Vector3 velocity
float64[9] velocity_covariance
