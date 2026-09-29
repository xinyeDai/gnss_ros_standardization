# ROS1 Noetic integration branch

This branch introduces a ROS1/catkin path for `gnss_ros_standardization`.

## Phase 1 scope

The first integration target is **solution-level (loosely coupled) GNSS**:

```
already-solved GNSS/PVT/RTK
  -> /gnss/external_solution  (ExternalGnssSolution)
  -> external_solution_adapter
  -> /gnss/solution           (GnssSolution)
  -> LIO/LIVO/PGO backend
```

The original raw-observation, ephemeris, SPP, RTK and FGO sources are retained in
the repository for later ROS1 migration, but they are not built by the Phase 1
catkin CMakeLists yet.

## Public solution interface

`GnssSolution.msg` is the downstream contract. It contains:

- ROS measurement timestamp and optional canonical GPS week/TOW
- standardized FIX/FLOAT/SINGLE/etc. status
- WGS84 BLH
- ECEF position/covariance
- ENU position/covariance and the ENU origin
- optional ECEF/ENU velocity and covariance
- quality fields such as satellite count, AR ratio, correction age and DOP

## External Solution Mode

`ExternalGnssSolution.msg` is deliberately smaller. An existing receiver,
GNSS/INS unit, proprietary driver or legacy ROS node can provide:

- timestamp
- WGS84 BLH
- 3x3 position covariance
- standardized status
- optional GPS week/TOW
- optional satellite count / ratio / age / DOP
- optional velocity and velocity covariance

The adapter performs BLH->ECEF, BLH->ENU, covariance-frame conversion and publishes
one normalized `GnssSolution`.

### ENU origin

For mapping and relocalization, use a fixed origin:

```yaml
auto_origin: false
origin_latitude: 30.0
origin_longitude: 114.0
origin_altitude: 30.0
```

For quick tests, `auto_origin: true` uses the first valid solution.

## Build

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch gnss_ros_standardization external_solution.launch
```

The downstream SLAM system should subscribe only to `/gnss/solution`.
