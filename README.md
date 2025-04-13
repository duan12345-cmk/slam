This repository provides the core implementation of **MSF-SLAM**, enhancing ORB-SLAM3 for dynamic environments with a YOLOv8-based Multi-Scale Feature Consolidation (MSFConv) module and Dynamic Object Filtering Framework (DOFF) using Lucas-Kanade optical flow. It achieves up to 94.43% improved pose accuracy on TUM RGB-D dynamic sequences.

Features
- YOLOv8 with MSFConv: 74.2% precision on VOC2007.
- DOFF: Dynamic feature removal with 1.5-pixel threshold.
- Real-time: 233.1 FPS (RTX 4060), 45.2 FPS (Jetson Xavier NX).

> Note: Due to memory constraints, only core code is included. Download ORB-SLAM3 separately, replace its `src` folder with this one, compile, and run.

Dependencies
- Ubuntu 18.04+
- C++14, CMake, OpenCV 4.2+, Eigen3, Pangolin
- Python 3.8+, PyTorch 1.8.0+
- NVIDIA GPU (recommended), CUDA, cuDNN

Installation
1. Download ORB-SLAM3:
   ```bash
   git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
   ```

2. Clone This Repository:
   ```bash
   git clone https://github.com/13183621726/Yolo-MSF-SLAM.git
   ```

3. Replace ORB-SLAM3 `src`:
   - Copy `Yolo-MSF-SLAM/src/` to `ORB_SLAM3/`, overwriting the original `src`.

4. Install Dependencies:
   ```bash
   sudo apt update
   sudo apt install build-essential cmake libopencv-dev libeigen3-dev libpangolin-dev python3-pip
   pip install torch==1.8.0 torchvision==0.9.0 numpy opencv-python
   ```

5. Build ORB-SLAM3:
   ```bash
   cd ORB_SLAM3
   chmod +x build.sh
   ./build.sh
   ``

 Usage
1. Download Datasets:
   - TUM RGB-D: [https://vision.in.tum.de/data/datasets/rgbd-dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)
   - VOC2007: [http://host.robots.ox.ac.uk/pascal/VOC/voc2007/](http://host.robots.ox.ac.uk/pascal/VOC/voc2007/)
   - Place in `ORB_SLAM3/data/`.

2. Run MSF-SLAM:
   ```bash
   ./Examples/Monocular/mono_tum \
       ./config/ORBvoc.txt \
       ./config/TUM3.yaml \
       ./data/rgbd_dataset_freiburg3_walking_xyz
   ```

 Results
| Dataset        | ATE RMSE (m) | Improvement |
|----------------|--------------|-------------|
| walking_xyz    | 0.0134       | 94.43%      |
| walking_half   | 0.0202       | 93.34%      |


