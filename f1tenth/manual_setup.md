# Manual Autoware Installation on NVIDIA Jetson

This tutorial provides step-by-step instructions for manually installing and setting up the Autoware development environment on an NVIDIA Jetson.

This guide assumes you are using `JetPack 6.2.1` on `Ubuntu 22.04` and will run `Autoware 1.6.0` on `ROS2 humble`.

The original [Autoware installation documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) from main branch is here for your reference.

The approximate time investments listed are based on running Jetson Orin Nano (Super) on the `MAXN SUPER` power mode.

## Set up Autoware dependencies
(Approximate time investment: 2-3 hours)

Some of the dependencies required for Autoware can't be or aren't installed automatically. These need to be set up manually.
1. Start by updating the system en ensuring the Jetpack packages are installed on your Jetson. This will install, among others, CUDA, CUDNN and TensorRT, which are required by Autoware.

    ```bash
    sudo apt update && sudo apt upgrade -y
    sudo apt install -y nvidia-jetpack
    sudo reboot
    ```

2. The version of OpenCV provided by NVidia does not contain all the functions required by Autoware. As such, we need to install an Autoware-compatible version. (Originally from [this guide](https://github.com/Ablazesphere/autoware_AGX)) 
    ```bash
    sudo apt remove --purge libopencv* opencv* python3-opencv

    sudo apt autoremove -y
    sudo apt autoclean

    sudo apt update

    sudo apt install -y libopencv-dev=4.5.4+dfsg-9ubuntu4 python3-opencv=4.5.4+dfsg-9ubuntu4
    ```

3. Autoware requires a specific c++ version of the Spatially Sparse Convolution Library (spconv) and it's dependency the CUda Matrix Multiply (cumm) library. On other systems, these will be installed automatically. On Jetson, these need to be built and installed manually.
    ```bash
    git clone -b spconv_v2.3.8+cumm_v0.5.3 https://github.com/autowarefoundation/spconv_cpp
    cd spconv_cpp

    mkdir -p cumm/build && cd cumm/build && cmake .. && make && cpack -G DEB
    cd ../../ && sudo apt install ./cumm/_packages/cumm_0.5.3_arm64.deb

    mkdir -p spconv/build && cd spconv/build && cmake .. && make -j $(nproc) && cpack -G DEB
    cd ../../ && sudo apt install ./spconv/_packages/spconv_2.3.8_arm64.deb
    ```

## Set up Autoware development environment 
(Approximate time investment: 0.5 hours)

1. Clone the `1.6.0` version of Autoware (`autowarefoundation/autoware`) and move to the directory.
    ```bash
    cd ~
    git clone -b 1.6.0 https://github.com/autowarefoundation/autoware.git
    cd autoware
    ```

2. Before running the setup script, it's `CRITICAL` to disable the agnocast task. Agnocast is currently incompatible with the default Linux Kernel version for the Jetson and will result in a `kernel panic` on next boot. You will need to reflash your Jetson and start over if this happens.

    Open up the Ansible playbook for editing:
    ```bash
    sudo apt install nano
    nano ansible/playbooks/universe.yaml
    ```

    Then, find and remove the following lines:
    ```bash
    - role: autoware.dev_env.agnocast
        when: rosdistro == 'humble'
    ```

    To save your changes, press `Ctrl+X` then `Y` and press `Enter`

3. Now, you can use the modified Ansible playbook to install the Autoware dependencies by running the provided setup script. Make sure to include the `--no-nvidia` and `--no-cuda-drivers` flags.

   ```bash
   ./setup-dev-env.sh --no-nvidia --no-cuda-drivers -y
   ```

   The NVIDIA library and cuda driver installation are disabled as they are already installed with the JetPack. If you force the CUDA driver installation here, it can mess up the kernel and cause errors at bootup. You will need to reflash the JetPack if this happens.

4. Lastly, make sure the CUDNN and TensorRT CMAKE modules are installed:
    ```bash
    sudo apt update 
    sudo apt install -y ros-humble-cudnn-cmake-module ros-humble-tensorrt-cmake-module
    ```


## Set up Autoware workspace 
(Approximate time investment: 3-4 hours)

1. Make sure you are in the previously created autoware directory
    ```bash
    cd autoware
    ```

2. Create the `src` directory and clone repositories into it.

   Autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces.

   ```bash
   mkdir src
   vcs import src < autoware.repos
   ```

3. Install dependent ROS packages.

    ```bash
    source /opt/ros/humble/setup.bash
    sudo apt update && sudo apt upgrade
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    ```

3. Create swapfile. (Originally from Autoware [troubleshooting section](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/#build-issues))

   Building Autoware requires a lot of memory. The Jetson can crash during a build because of insufficient memory. To avoid this problem, 16-32GB of swap should be configured.

   Optional: Check the current swapfile
   ```bash
   free -h
   ```
   
   Create a new swapfile
   ```bash
   sudo fallocate -l 32G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

   Optional: Check if the change is reflected
   ```bash
   free -h
   ```

   Optional: To make this change permanent
   ```bash
   sudo bash -c 'echo "/swapfile swap swap defaults 0 0" >> /etc/fstab'
   ```

4. Build the workspace.

   Autoware uses [colcon](https://github.com/colcon) to build workspaces.
   For more advanced options, refer to the [documentation](https://colcon.readthedocs.io/).

   ```bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   Ignore the `stderr` warnings during the build.