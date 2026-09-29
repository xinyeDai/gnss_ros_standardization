# gnss_ros_standardization — ROS1 Noetic branch

This branch provides a ROS1/catkin GNSS standardization layer with **two input
modes and one common solution interface**.

```
Mode A: Raw GNSS
receiver / RTCM byte stream
        |
        v
raw_gnss_decoder
        |
        +--> GnssObservations
        +--> GnssEphemerides
                 |
          +------+------+
          |             |
          v             v
         SPP            RTK
          |             |
          +------+------+
                 v
            GnssSolution

Mode B: External solved GNSS
timestamp + WGS84 BLH + covariance + status (+ optional velocity)
                 |
                 v
       ExternalGnssSolution
                 |
                 v
   external_solution_adapter
                 |
                 v
            GnssSolution
```

Downstream LIO/LIVO/SLAM systems should consume `GnssSolution` regardless of
where the GNSS solution came from.

## ROS1 nodes

| Node | Role |
|---|---|
| `raw_gnss_decoder` | Decode UBX, Septentrio SBF, NovAtel OEM or RTCM3 streams into standardized raw observations/ephemerides |
| `single_point_positioning` | RTKLIB SPP from `GnssObservations + GnssEphemerides` |
| `real_time_kinematic` | RTKLIB rover/base RTK from standardized raw observations |
| `external_solution_adapter` | Normalize an already-computed GNSS/PVT/RTK solution |

## Public interfaces

Raw-mode interfaces:

- `GnssObservation.msg`
- `GnssObservations.msg`
- `GnssEphemeris.msg`
- `GlonassEphemeris.msg`
- `GnssEphemerides.msg`

Solution-level interfaces:

- `ExternalGnssSolution.msg` — input contract for already-solved GNSS data
- `GnssSolution.msg` — common downstream output

The common solution carries GNSS time, standardized solution status, BLH, ECEF,
ENU, covariance, velocity and quality indicators.

## Build

Ubuntu 20.04 + ROS Noetic:

```bash
cd ~/catkin_ws/src
git clone --recursive -b ros1-noetic https://github.com/xinyeDai/gnss_ros_standardization.git
cd gnss_ros_standardization
git submodule update --init --recursive

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

The RTKLIB submodule is required for Raw Mode.

## Mode A — raw observation processing

### SPP

Configure `config/raw_gnss_decoder.yaml` for the receiver stream, then:

```bash
roslaunch gnss_ros_standardization raw_spp.launch
```

Data path:

```
receiver raw stream
 -> /gnss/observation
 -> /gnss/ephemeris
 -> single_point_positioning
 -> /gnss/solution
```

### RTK

Example configs are provided for a raw rover receiver and an RTCM/NTRIP base:

```bash
roslaunch gnss_ros_standardization raw_rtk.launch
```

Data path:

```
rover raw -> /rover/gnss/observation ----+
                                          +-> real_time_kinematic -> /gnss/solution
base RTCM -> /base/gnss/observation -----+
          -> /base/gnss/station_ecef
rover/base -> /gnss/ephemeris
```

Supported decoder formats in the ROS1 generic raw node:

- u-blox UBX
- Septentrio SBF
- NovAtel OEM
- RTCM3

Streams are opened through RTKLIB and can be serial, file, TCP client or NTRIP
client.

## Mode B — external solved solution

```bash
roslaunch gnss_ros_standardization external_solution.launch
```

Input:

```
/gnss/external_solution
  timestamp
  WGS84 latitude / longitude / ellipsoidal height
  position covariance
  FIX / FLOAT / SINGLE / ...
  optional GPS week/TOW
  optional velocity
  optional quality fields
```

Output:

```
/gnss/solution
```

The adapter performs BLH->ECEF, BLH->ENU and covariance-frame normalization.

## ENU origin

Both modes can publish the same fixed ENU frame. For globally repeatable mapping
and relocalization, configure a fixed WGS84 ellipsoidal BLH origin instead of a
first-fix origin.

The ENU origin is a coordinate-system definition. It is not the same as the RTK
base-station coordinate and it is not the initial SLAM pose.

## Current ROS1 scope

The core raw-observation pipeline is implemented in ROS1 using a generic RTKLIB
decoder instead of mechanically porting every ROS2 brand-specific driver.

Therefore the ROS1 branch currently provides the essential raw functionality:

```
raw bytes -> observations/ephemerides -> SPP/RTK -> GnssSolution
```

The following ROS2 driver extras are **not yet feature-parity ports**:

- automatic receiver configuration commands
- receiver-specific binary PVT fan-out
- receiver IMU outputs
- receiver-specific NMEA aggregation/fan-out
- rosbag2/RINEX converter tooling
- tightly coupled GTSAM FGO examples

Those components remain in the repository history/main branch and can be ported
independently without changing the public ROS1 raw/solution interfaces.

## Important runtime rule

Run only the solution producer that corresponds to the selected mode. Do not
publish multiple independent producers to `/gnss/solution` simultaneously
unless an explicit mux/arbitration layer is added.

## License

MIT. The vendored RTKLIB fork retains its own upstream license.
