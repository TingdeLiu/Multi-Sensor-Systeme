<p align="center">
<h1 align="center"><strong>📡 Multi-Sensor Systems – Exercises (WiSe 2022/2023)</strong></h1>
  <p align="center">
    <a href='mailto:khami@gih.uni-hannover.de' target='_blank'>M. Sc. Arman Khami</a>&emsp;
    <a href='mailto:tingde.liu@gmail.com' target='_blank'>Tingde Liu</a>&emsp;
    <br>
    Geodätisches Institut Hannover, Leibniz Universität Hannover
  </p>
</p>

---

## 🧭 Overview

This is a course I took at Leibniz University and the code is an exercise for my accompanying course. This repository documents three practical exercises from the lecture **Multi-Sensor Systems (MSS)** in the winter semester 2022/2023. The exercises are designed to deepen theoretical concepts using MATLAB and ROS environments in a fully online setup.

---

## 🧪 Exercise Summary

### 🔧 1. ROS Basics

- Platform: **ILIAS**
- Familiarization with:
  - ROS structure and packages
  - Topics, services, nodes, launch files
- External resources:
  - [ROS Wiki](http://wiki.ros.org/)
  - [Clearpath ROS Guide](http://www.clearpathrobotics.com/assets/guides/kinetic/ros/)

---

### 📐 2. Synchronization & Extrinsic Calibration of IMU

- Platform: **MATLAB Grader**
- Tasks:
  - Synchronize IMU and camera data
  - Prepare datasets and align timestamps
  - Perform extrinsic calibration using **Iterative Extended Kalman Filter (IEKF)**
- Calibration includes:
  - IMU ↔ Platform
  - IMU ↔ Camera
- Based on:
  - Mirzaei et al., IEEE TRO 2008
  - Master thesis by Jiayu Liu (2021)

---

### 🌍 3. Georeferencing & Validation of a k-TLS-based System

- Platform: **MATLAB Grader**
- Tasks:
  - Identify individual scan blocks
  - Compute 3D rotation and transformation matrices
  - Interpolate laser tracker poses
  - Georeference each scan block
  - Validate kinematic point cloud via comparison to static reference plane
- Visualization:
  - Histogram of point-to-plane distances
  - 3D point cloud overlays

---

## 🛠 Platform & Tools

- ROS Noetic (Ubuntu 20.04)
- MATLAB Grader (Online MATLAB)
- Dataset preprovided, no field measurement required

---

## 📅 Course Info

| Übung | Thema                                                   | Plattform       |
|--------|----------------------------------------------------------|------------------|
| 1      | ROS Grundlagen                                           | ILIAS            |
| 2      | Synchronisation & Extrinsische Kalibrierung einer IMU   | MATLAB Grader    |
| 3      | Georeferenzierung & Validierung                         | MATLAB Grader    |

- No group work
- Unlimited submissions
- Minimum score per task required
- Final colloquium mandatory

---

## 📬 Contacts

- **Supervisor**: Dr.-Ing. Sören Vogel – [vogel@gih.uni-hannover.de](mailto:vogel@gih.uni-hannover.de)
- **Instructor**: M. Sc. Arman Khami – [khami@gih.uni-hannover.de](mailto:khami@gih.uni-hannover.de)
- **HiWi Support**: Nathanael Hehs – [nathanael.hehs@stud.uni-hannover.de](mailto:nathanael.hehs@stud.uni-hannover.de)

---

## 📎 References

- Mirzaei, F. M., & Roumeliotis, S. I. (2008). A Kalman Filter-Based Algorithm for IMU-Camera Calibration.
- Liu, Jiayu (2021). *Extrinsische Kalibrierung einer IMU für ein Multi-Sensor System*, Masterarbeit, LUH.
- Ernst, D. (2021). *Quality model for uncertainty judgement of kinematic TLS-MSS*, Masterarbeit, LUH.

---

<p align="center"><i>“Reliable sensor fusion starts with precise calibration.”</i></p>
"""

# Save to file
output_path = "/mnt/data/README_MSS_LabSeries.md"
with open(output_path, "w", encoding="utf-8") as f:
    f.write(readme_content.strip())

output_path
