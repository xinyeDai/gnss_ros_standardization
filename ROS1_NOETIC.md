# ROS1 Noetic design

## Goal

The ROS1 branch is intentionally a **dual-input GNSS frontend**.

### Raw Observation Mode

```
UBX / SBF / NovAtel OEM / RTCM3
        |
        v
raw_gnss_decoder
        |
        +--> GnssObservations
        +--> GnssEphemerides
                 |
           SPP or RTK
                 |
                 v
            GnssSolution
```

This path preserves the package's original purpose: exposing standardized raw
GNSS observations and ephemerides and optionally computing SPP/RTK locally.

### External Solution Mode

```
existing receiver / GNSS-INS / proprietary positioning program
        |
timestamp + BLH + covariance + status
        |
        v
ExternalGnssSolution
        |
external_solution_adapter
        |
        v
GnssSolution
```

This path is for systems where GNSS has already been solved upstream.

## Why both modes use GnssSolution

The downstream SLAM backend should not depend on receiver brand or on whether
position was computed by RTKLIB, by a commercial receiver, or by another GNSS
program.

`GnssSolution` is therefore the single position-level contract.

For future tightly coupled GNSS/LIO work, the raw interfaces remain available:
`GnssObservations + GnssEphemerides` can be consumed directly instead of
`GnssSolution`.

## ROS1 executables

- `raw_gnss_decoder`
- `single_point_positioning`
- `real_time_kinematic`
- `external_solution_adapter`

## Coordinate policy

`GnssSolution` includes WGS84 BLH, ECEF, ENU and the ECEF coordinate of the
ENU origin.

A fixed ENU origin is recommended for global-map/relocalization use. Both Raw
and External modes must be configured with the same origin when their outputs
need to be interchangeable.

RTK base-station position is a different concept: it is used by RTKLIB to form
the differential solution and does not define the SLAM global frame unless the
user deliberately chooses the same coordinate as the ENU origin.

## Time policy

- `header.stamp`: measurement timestamp used by downstream ROS sensor fusion
- `time_week/time_tow`: canonical GPST when available
- raw decoders can stamp by ROS receive time or GPST-derived UTC
- SPP/RTK preserve the rover observation stamp
- External Mode preserves the upstream input stamp and validates GPST if supplied

## Covariance policy

All public covariance matrices are variances/covariances, not standard
deviations.

- `pos_cov_ecef`: m^2 in ECEF
- `pos_enu_cov`: m^2 in the local ENU tangent frame
- velocity covariance: (m/s)^2

External Mode accepts either ENU or ECEF covariance and produces both.

## Current boundary

ROS1 Raw Mode is implemented through RTKLIB's generic stream and raw decoders.
It is not yet a line-for-line port of every ROS2 receiver driver feature. The
receiver must already be configured to output the required raw messages.

This boundary keeps the core interface stable while later receiver-specific
configuration, PVT, IMU, NMEA and converter features are ported separately.
