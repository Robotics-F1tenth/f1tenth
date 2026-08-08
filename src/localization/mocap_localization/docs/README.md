# Two-vehicle mocap documentation

Start with [two_vehicle_mocap_pipeline.md](two_vehicle_mocap_pipeline.md). It is the
single runbook for:

- Motive and the Victus OptiTrack bridge laptop
- ROS domain bridging for car 3 and car 4
- simultaneous vehicle-specific mocap poses
- shared-map and vehicle-specific calibration
- frame-delivery diagnosis
- experiment bag collection and evaluation quality gates

## Repository changes made during the 2026-08-07 audit

- `mocap_localization.cpp` now preserves `/rigid_bodies.header.stamp` in the
  transformed pose and TF.
- The per-frame yaw `RCLCPP_INFO` was replaced by an opt-in, one-second-throttled
  INFO diagnostic.
- `child_frame_id` is configurable to avoid TF collisions between two vehicles.
- `mocap_localization_all.launch.py` launches both rigid-body consumers together.
- `mocap_timing_probe.py` reports live source/receive timing once per second.
- `mocap_bag_timing_report.py` separates measurement-header timing, observed driver
  sequence gaps, and delivery batching.
- `calibrate_shared_map.py` fits one scale-free surveyed map transform and writes two
  protected candidate YAML files with byte-for-byte-equal shared transform values.
- The legacy `calibration_v2.py` writes a candidate by default instead of resetting
  production parameters. Its six-parameter fit is still underconstrained and is not
  the final calibration method.
- Four directional domain-bridge examples, rosbag QoS, and an experiment manifest
  are under `config/`.
- `patches/mocap4r2_optitrack_driver_low_latency.patch` fixes the upstream rolling
  publisher's hardcoded `RELIABLE/KeepLast(1000)` policy for an A/B test. The QoS
  keys shipped in that upstream YAML are not consumed by its C++ implementation.
- `patches/mocap4r2_optitrack_driver_timing_diagnostics.patch` adds one aggregate
  timing line per second for native NatNet frames, callback spacing, DDS publish,
  and total callback work.

These changes are in `/home/a/f1tenth`. The Victus laptop previously showed a running
binary from `/home/a/Downloads/mocap_ws/install`; it is not updated until this package is
copied/merged there, rebuilt, sourced, and restarted.
