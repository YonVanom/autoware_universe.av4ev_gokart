# Roboracer × ZED X
### Setup & Development Guide — Orin Nano + ZED Link Duo

---

## Table of Contents

- [Hardware Setup](#hardware-setup)
- [Capture Card & SDK Setup](#capture-card--sdk-setup)
- [Testing the ZED X](#testing-the-zed-x)
- [Using the ZED SDK API](#using-the-zed-sdk-api)
- [Using the ZED SDK Samples](#using-the-zed-sdk-samples)
- [ZED ROS 2](#zed-ros-2)
- [ZEDfu](#zedfu)

---

## Hardware Setup

Connect the ZED X camera to the Orin Nano via the capture card. Full hardware wiring steps are covered in the official Stereolabs guide:

> 📖 [ZED Link Dual Jetson Orin Nano DevKit Setup](https://www.stereolabs.com/docs/embedded/zed-link/dual-jetson-orin-nano-devkit-setup)

---

## Capture Card & SDK Setup

### 1 — Connect the Capture Card

Follow the hardware connection steps for the **ZED Link Duo on Orin Nano**:

> 📖 [ZED Link Orin Nano Setup](https://www.stereolabs.com/docs/embedded/zed-link/dual-jetson-orin-nano-devkit-setup)

### 2 — Install Drivers

Download the correct driver for your capture card. To find which driver you need:

> 📖 [Driver Installation Guide](https://www.stereolabs.com/docs/embedded/zed-link/install-the-drivers)  
> 📦 [Driver Downloads](https://www.stereolabs.com/developers/drivers)

Follow the installation steps through to completion.

### 3 — Install the ZED SDK

Download the correct SDK version for your Jetson:

> 📦 [SDK Downloads](https://www.stereolabs.com/developers/release)  
> 📖 [SDK Installation Guide — Jetson](https://www.stereolabs.com/docs/development/zed-sdk/jetson)

---

## Testing the ZED X

Once drivers and SDK are installed, verify everything is working:

**1. Check the daemon is running**
```bash
sudo systemctl status zed_x_daemon
```

**2. Verify the camera is detected**
```bash
sudo dmesg | grep zedx
```
> ⚠️ Returns nothing if no camera is connected — that's expected with no hardware attached.

**3. Open the Depth Viewer**
```bash
cd /usr/local/zed/tools
./ZED_Depth_Viewer
```
This opens an interactive GUI showing the live camera feed, depth map, and 3D point cloud.

---

## Using the ZED SDK Samples

Prebuilt samples live in `/usr/local/zed/samples`. Each sample has both a Python and C++ version.

**1. Navigate to the samples directory**
```bash
cd /usr/local/zed/samples
```

**2. Select a sample and navigate into it**

The folder structure looks like:
```
/usr/local/zed/samples/
└── positional tracking/
    └── positional tracking/
        └── cpp/
```

**3. Build the sample**

Follow the build steps in the relevant quickstart guide, e.g.:

> 📖 [Positional Tracking Quickstart](https://www.stereolabs.com/docs/positional-tracking/quickstart)

```bash
cd cpp
mkdir build && cd build
cmake ..
make
```

**4. Run the executable**
```bash
./ZED_Positional_Tracking [args]
```

Refer to each sample's quickstart guide for available arguments.

---

## ZED ROS 2

> 📖 [ZED ROS 2 Documentation](https://www.stereolabs.com/docs/ros2)  
> 📦 [ZED ROS 2 Wrapper — GitHub](https://github.com/stereolabs/zed-ros2-wrapper)

---

## ZEDfu

ZEDfu is the Stereolabs tool for working with recorded data. Use it to:

- Visualize `.svo` / `.svo2` recording files
- View and export 3D mesh
- Convert recordings to mesh format

```bash
/usr/local/zed/tools/ZEDfu
```
