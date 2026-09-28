# DRDO UAV-Guided UGV Navigation System Documentation

Welcome to the technical documentation directory for the **DRDO UAV-Guided UGV Autonomous Navigation System**.

This documentation suite provides a complete, submission-ready explanation of the collaborative aerial-ground robotic architecture, sensor pipelines, computer vision models, pure pursuit control mechanics, and code-level directory walkthroughs.

---

## Documentation Index

| Document | Topic & Focus Area |
| :--- | :--- |
| [**1. System Overview & Architecture**](01_SYSTEM_OVERVIEW_AND_ARCHITECTURE.md) | Mission concept, 3-phase execution cycle, system topology, ROS 2 / Gazebo / PX4 communication boundaries, and REP-103/105 coordinate frame trees. |
| [**2. Sensor Stack & Depth Processing Pipeline**](02_SENSOR_STACK_AND_DEPTH_PIPELINE.md) | Physical OakD-Lite hardware vs. Gazebo simulation SDF, nadir pitch mounting, why raw depth images look black (`32FC1`), byte decoding, pinhole back-projection, and 3D point cloud generation. |
| [**3. 2.5-D Mapping & Costmap Generation**](03_MAPPING_CLASSIFICATION_AND_COSTMAP.md) | Cumulative elevation grid updates, local tangent plane fitting, geometric hazard evaluation (slope, step, roughness), morphological costmap shaping, and multi-level switchback slicing. |
| [**4. Guidance, Localization & Pure Pursuit Control**](04_GUIDANCE_LOCALIZATION_AND_CONTROL.md) | Overhead ArUco UGV tracking at 10m AGL, mathematical breakdown of the custom zero-dependency Regulated Pure Pursuit controller, curvature formulas, in-place pivot guards, and autonomous frontier survey. |
| [**5. Code-Level File Directory**](05_CODE_LEVEL_FILE_DIRECTORY.md) | **Exhaustive file-by-file walkthrough** covering inputs (topics, QoS, params), internal algorithms, and outputs for every source file, launch script, and model in the entire repository. |
| [**6. Reference Repositories & Prior Art**](06_REFERENCE_REPOSITORIES_AND_PRIOR_ART.md) | Architectural attribution, upstream dependencies (`PX4-Autopilot`, `Micro-XRCE-DDS-Agent`, `gazebosim/gz-sim`, `ros_gz`), comparisons with `Nav2 Regulated Pure Pursuit`, and academic literature citations. |

---

## Architectural Diagrams

The documentation references four high-resolution architectural figures located in this directory:
- [`fig_topology.png`](fig_topology.png): Complete system hardware and software network topology.
- [`fig_sensor_flow.png`](fig_sensor_flow.png): Gazebo sensor buffer generation, bridging, decoding, and mapping pipeline.
- [`fig_frames.png`](fig_frames.png): Coordinate transformation tree connecting PX4 NED, ROS ENU, and CV Optical frames.
- [`fig_wheel_fix.png`](fig_wheel_fix.png): Physical chassis kinematics and wheel contact dynamics in Gazebo Harmonic.
