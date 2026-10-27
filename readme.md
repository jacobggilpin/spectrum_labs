# Multi-Camera 3D LiDAR Point Cloud Registration & Reconstruction (Intel RealSense L515)

A MATLAB pipeline for multi-view 3D point cloud capture, ground-plane leveling, spatial calibration, and coarse-to-fine registration using **three Intel RealSense L515 LiDAR cameras** mounted on a custom **6-foot wooden equilateral triangular prism frame**.

## Project Overview

Single-view depth sensors suffer from self-occlusion, perspective shadowing, and uneven point density when scanning 3D objects. During this summer engineering project, a custom **open wooden triangular prism rig** was designed and constructed to hold three Intel RealSense L515 solid-state LiDAR cameras spaced **120° apart** around a central scanning workspace.

Because physical wooden framing and camera mounting brackets introduce slight angular pitch, roll, and translation tolerances relative to the nominal 120° geometry, raw point clouds cannot be merged using idealized rotations alone. This repository implements a **two-stage Calibration + Alignment architecture**:

1. **Calibration Mode (`'calibrate'`)**: Scans a simple geometric reference object (such as a calibration box) from all three cameras, levels the ground plane onto $Z = 0$, applies the nominal $0^\circ / +120^\circ / -120^\circ$ azimuthal rotations, removes floor points using an adaptive height threshold, segments the calibration target via Euclidean clustering (`pcsegdist`), and computes camera-to-camera transformations using Plane-to-Plane Iterative Closest Point (`pcregistericp`) followed by Rigid Coherent Point Drift (`pcregistercpd`). The resulting base transformations are saved to `calibration_data.mat`.
2. **Alignment Mode (`'align'`)**: Loads the pre-computed `calibration_data.mat` transformations as a base spatial adjustment for new 3-camera scans, isolates the target object cluster while preserving RGB color and surface normals, and executes a **secondary fine ICP pass** to eliminate residual pose drift before merging the three views into a unified, high-density 3D model (`pcmerge`).

## Hardware Setup & Frame Geometry

| Component / Parameter | Specification | Engineering Notes |
| :--- | :--- | :--- |
| **Sensors** | 3 × Intel RealSense L515 | Solid-state MEMS LiDAR depth camera (640×480 @ 30 FPS depth + BGR8 color) |
| **Frame Topology** | Open Equilateral Triangular Prism | Unfilled wooden timber frame providing unobstructed 360° inward sightlines |
| **Vertical Frame Height** | ~6.0 ft (1.83 m) | Three vertical wooden legs supporting elevated downward-angled camera mounts |
| **Equilateral Side Length** | ~3.5 ft (1.07 m) | Triangular footprint circumradius $R = L/\sqrt{3} \approx 2.02\text{ ft}$ ($0.616\text{ m}$) from center |
| **Azimuthal Leg Spacing** | $120^\circ$ ($0^\circ, 120^\circ, 240^\circ$) | Symmetric 3-way coverage around the central floor workspace |
| **Sensor Viewing Pitch** | ~25°–40° from nadir | Slant range $d \approx 1.88\text{ m}\text{–}1.97\text{ m}$ to the floor center; depth filter window $[0.1\text{ m}, 3.0\text{ m}]$ |

## Repository Structure

```text
├── lidar_point_cloud_alignment.m          # Main refactored MATLAB pipeline (Calibrate, Align, Both, Capture)
├── README.md                              # GitHub repository overview and quick-start guide
├── Summer_Lidar_Project_Documentation.pdf # Full engineering report, rig diagrams, and summer retrospective
├── Summer_Lidar_Project_Documentation.md  # Markdown version of the full technical documentation
└── data/                                  # Included calibration, synthetic, and real L515 scan datasets
    ├── example_box_angle_0.ply            # Low-poly synthetic unit cube at 0° (8 vertices, 6 faces)
    ├── example_box_angle_120.ply          # Low-poly synthetic unit cube rotated +120° (8 vertices, 6 faces)
    ├── example_box_angle_240.ply          # Low-poly synthetic unit cube rotated +240° / -120° (8 vertices, 6 faces)
    ├── dense_box_angle_0.ply              # High-density calibration box at 0° (98,306 vertices with normals)
    ├── dense_box_angle_120.ply            # High-density calibration box at 120° (98,306 vertices with normals)
    ├── dense_box_angle_240.ply            # High-density calibration box at 240° (98,306 vertices with normals)
    ├── f1.ply                             # RealSense L515 capture (Low Quality) from Camera 1 (287,160 RGB-D vertices)
    ├── f2.ply                             # RealSense L515 capture (Low Quality) from Camera 2 (272,949 RGB-D vertices)
    └── f3.ply                             # RealSense L515 capture (Low Quality) from Camera 3 (266,610 RGB-D vertices)
```

## Included Datasets

| Dataset Group | Files | Format | Vertex Count (per file) | Attributes | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Synthetic Unit Cube** | `example_box_angle_0.ply`, `120.ply`, `240.ply` | ASCII PLY | 8 vertices, 6 faces | `x, y, z` | Lightweight geometric verification of exact $0^\circ / 120^\circ / 240^\circ$ rotations |
| **Dense Calibration Box** | `dense_box_angle_0.ply`, `120.ply`, `240.ply` | Binary PLY | 98,306 vertices | `x, y, z, nx, ny, nz` | High-density surface-normal calibration target for `planeToPlane` ICP & CPD |
| **RealSense L515 Captures** | `f1.ply`, `f2.ply`, `f3.ply` | ASCII PLY | 266,610 – 287,160 | `x, y, z, r, g, b` | Real multi-camera L515 hardware captures of a central box/pedestal on the lab floor |

## Requirements

- **MATLAB** (R2022b or newer recommended for `rigidtform3d`)
- **Computer Vision Toolbox** (`pcread`, `pcwrite`, `pctransform`, `pcregistericp`, `pcregistercpd`, `pcsegdist`, `pcmerge`, `findPointsInROI`, `pcfitplane`)
- **Intel RealSense SDK 2.0 (`librealsense2`) MATLAB Wrapper** *(Required only when running live hardware capture via `'Mode', 'capture'`)*

## Quick Start & Usage

### 1. Run Full Pipeline (Calibrate + Align in One Command)

By default, running `lidar_point_cloud_alignment` executes **Calibration Mode** on `dense_box_angle_0/120/240.ply`, saves `calibration_data.mat`, and immediately runs **Alignment Mode** on `f1.ply`, `f2.ply`, and `f3.ply`:

```matlab
results = lidar_point_cloud_alignment();
```

### 2. Run Calibration Mode Only

To compute and store the base 3D transformations from a calibration target:

```matlab
calibResults = lidar_point_cloud_alignment( ...
    'Mode', 'calibrate', ...
    'CalibFiles', {'dense_box_angle_0.ply', 'dense_box_angle_120.ply', 'dense_box_angle_240.ply'}, ...
    'CalibrationFile', 'calibration_data.mat', ...
    'GroundHeightDivisor', 8.0, ...
    'UseCPDInCalibration', true);
```

### 3. Run Alignment Mode Only (Using Saved Calibration + Secondary ICP)

Once `calibration_data.mat` is generated, align any new 3-camera scan set using the calibrated base transforms followed by secondary fine ICP refinement:

```matlab
alignResults = lidar_point_cloud_alignment( ...
    'Mode', 'align', ...
    'TargetFiles', {'f1.ply', 'f2.ply', 'f3.ply'}, ...
    'CalibrationFile', 'calibration_data.mat', ...
    'OutputMergedPly', 'merged_aligned_model.ply', ...
    'AlignICPInlierRatio', 0.85, ...
    'MergeGridStep', 0.001);
```

### 4. Live Capture + Alignment from 3 Intel RealSense L515 Cameras

When connected to the three L515 cameras over USB 3.1:

```matlab
liveResults = lidar_point_cloud_alignment( ...
    'Mode', 'capture', ...
    'CalibrationFile', 'calibration_data.mat', ...
    'OutputMergedPly', 'live_scan_merged.ply');
```

## Key Script Refinements & Bug Fixes

Compared to the original prototype script, `lidar_point_cloud_alignment.m` introduces the following structural and mathematical upgrades:

- **Separated Calibration & Alignment Modes**: Modularized the script into `runCalibrationMode` (which exports a persistent `calibration_data.mat` struct containing ground, nominal, ICP, and CPD transforms) and `runAlignmentMode` (which loads `calibration_data.mat` as a base adjustment and performs secondary fine ICP refinement on new scans).
- **Fixed Post-Calibration Transform Copy-Paste Bug**: In the original script's post-calibration block, Camera 1's ground transform (`transformation1A`) and $0^\circ$ rotation (`transformation1B`) were accidentally applied to Cameras 2 and 3 (`pctransform(pointCloud2, transformation1A)` and `pctransform(pointCloud2AA, transformation1B)`). The refactored pipeline applies each camera's dedicated transformation chain (`tformGround{i}`, `tformNominal{i}`, `tformCalibICP{i}`, `tformCalibCPD{i}`).
- **Implemented Dynamic Ground Thresholding (`maxZ / X`)**: Fulfilled the inline code note (`% ADJUST SO ITS BASED ON MAX Z HEIGHT DIVIDED BY X`) via `removeGroundAdaptive()`, which computes `threshold = max(MinGroundThreshold, maxZ / GroundHeightDivisor)`.
- **Preserved RGB Colors and Surface Normals**: Replaced raw coordinate array reconstruction (`pointCloud(objectPointsFiltered)`) with `select(ptCloud, indices)` so RGB vertex colors (in `f1.ply`–`f3.ply`) and surface normals (in `dense_box_angle_*.ply`) are preserved through ground removal, clustering, and merging.
- **Replaced Internal `lidarViewer` Crop Helper**: Replaced the undocumented `lidar.lidarViewer.EditPcCrop()` call with standard `findPointsInROI()` and `select()`, eliminating version-specific dependencies while preserving exact ROI bounds (`X: [-0.5, 0.5]`, `Y: [-0.75, 0.75]`, `Z: [-0.34799, 1.0407]`).
- **Added Multi-Frame Hardware Warm-Up & Color Mapping**: Enhanced `capture_point_clouds()` to run a 15-frame warm-up loop so the RealSense temporal and spatial filters stabilize before exporting RGB-mapped `.ply` files.
