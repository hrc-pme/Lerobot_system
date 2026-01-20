# ========================================================
# CUDA 12.8 with Ubuntu 22.04
# ROS2 Humble
# PyTorch cu 12.8
# Python 3.10
# ========================================================
FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

# ===================================================================       
# Locale & non root user
# ===================================================================
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# ===================================================================
# Timezone
# ===================================================================
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime

# ===================================================================
# NVIDIA Runtime
# ===================================================================
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# ==================================================================
# Install basic packages (avoid CUDA repo issues)
# ===================================================================
RUN apt-get update --allow-releaseinfo-change && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential cmake ninja-build pkg-config gdb \
    # System basics
    curl wget git vim nano less sudo unzip \
    net-tools iputils-ping socat \
    gnupg2 dirmngr lsb-release software-properties-common \
    tzdata locales ca-certificates \
    # Python
    python3 python3-pip python3-venv python3-dev python3-setuptools python3-wheel \
    # Media/Graphics
    libjpeg-dev libpng-dev libtiff5-dev zlib1g-dev libbz2-dev liblzma-dev \
    mesa-utils libegl1 freeglut3-dev libglu1-mesa-dev libglfw3-dev \
    libgtk-3-dev \
    # USB/Udev
    libusb-1.0-0-dev libudev-dev udev \
    # OpenCV (system) - for ROS cv_bridge/vision_opencv stability
    libopencv-dev python3-opencv \
    # GStreamer
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    # Qt/X11
    qtbase5-dev libqt5x11extras5-dev libcanberra-gtk-module libcanberra-gtk3-module \
    x11-xserver-utils xdotool wmctrl xterm \
    # FFmpeg
    ffmpeg \
    libavcodec-dev libavformat-dev libavutil-dev \
    libswscale-dev libavdevice-dev libavfilter-dev \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

# ===================================================================
# ROS 2 Humble KEY & Source
# ===================================================================
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros2-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ===================================================================
# Install ROS 2 
# ===================================================================
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
      software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-rosbridge-server \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-tools \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-setup-assistant \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        python3-colcon-common-extensions \
        python3-argcomplete \
        python3-rosdep \
    && rosdep init || true \
    && rosdep update || true \
    && rm -rf /var/lib/apt/lists/*

# ===================================================================
# RealSense 2.56.5
# ===================================================================
ENV LIBREALSENSE_VERSION=2.56.5
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl-dev pkg-config \
    && git clone -b v${LIBREALSENSE_VERSION} https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && mkdir -p /tmp/librealsense/build && cd /tmp/librealsense/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/librealsense \
    && rm -rf /var/lib/apt/lists/*

# ===================================================================
# Python venv (shared with system site-packages for ROS)
# ===================================================================
ENV VENV_PATH=/opt/venv

# Add venv site-packages to PYTHONPATH so system python can see venv packages
ENV PYTHONPATH=${VENV_PATH}/lib/python3.10/site-packages

RUN python3 -m venv ${VENV_PATH} --system-site-packages && \
    ${VENV_PATH}/bin/pip install --upgrade pip setuptools wheel


# Make venv default python/pip
ENV PATH=${VENV_PATH}/bin:$PATH

# ===================================================================
# LeRobot (source install, pinned ref)
# ===================================================================
# Installed AFTER venv is set up so it goes into the venv
ARG LEROBOT_REF=lerobot-system
RUN git clone --branch ${LEROBOT_REF} https://github.com/LesterLiou/lerobot.git /opt/lerobot && \
    cd /opt/lerobot && \
    pip install --no-cache-dir -e . && \
    pip install --no-cache-dir -U huggingface_hub rerun-sdk wandb

# ===================================================================
# Python pkg (installed into venv)
# ===================================================================
RUN python3 -m pip uninstall -y torch torchvision torchaudio || true \
    && pip install --upgrade pip \
    && pip install --no-cache-dir   torch==2.7.1 torchvision==0.22.1 torchaudio==2.7.1 --index-url https://download.pytorch.org/whl/cu128 \
    && pip install --no-cache-dir \
        numpy==1.26.4 matplotlib==3.8.* scipy pandas \
        scikit-learn scikit-image Pillow \
        pyserial pexpect future pyquaternion pyyaml \
        anyskin pygame dynamixel-sdk



# ===================================================================
# ADD  
# ===================================================================

    
# ===================================================================
# Create non-root user and setup workspace  
# ===================================================================
# Accept build arguments for dynamic user creation
ARG NB_USER=hrc
ARG NB_UID=1000
ARG NB_GID=1000

RUN groupadd -g ${NB_GID} ${NB_USER} \
    && useradd -m -s /bin/bash -u ${NB_UID} -g ${NB_GID} ${NB_USER} \
    && usermod -aG dialout,video,plugdev ${NB_USER} \
    && echo "${NB_USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/99-${NB_USER} \
    && chmod 0440 /etc/sudoers.d/99-${NB_USER}

# ===================================================================
# Setup ROS2 workspace (without copying source code)
# ===================================================================
USER ${NB_USER}
ENV HOME=/home/${NB_USER}
WORKDIR ${HOME}

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc && \
    echo "source /opt/venv/bin/activate" >> ${HOME}/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ${HOME}/.bashrc && \
    echo "export CUDA_VISIBLE_DEVICES=0" >> ${HOME}/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\${LD_LIBRARY_PATH}" >> ${HOME}/.bashrc && \
    echo "" >> ${HOME}/.bashrc && \
    echo "# Useful aliases for ROS2 development" >> ${HOME}/.bashrc && \
    echo "alias cb='cd ~/Lerobot_system/ros2_ws && rm -rf install build log && colcon build --merge-install && source install/setup.bash'" >> ${HOME}/.bashrc

# Set default working directory to user home (not ros2_ws)
WORKDIR ${HOME}

# Expose ROS2 ports
# 11311: ROS1 Master (for compatibility)
# 11345: ROS2 default discovery port  
# 7400-7500: ROS2 DDS communication range
EXPOSE 11311 11345 7400-7500
# ===================================================================
# Entrypoint: auto source ROS + venv
# ===================================================================
COPY ros_venv_entrypoint.sh /ros_venv_entrypoint.sh

ENTRYPOINT ["/ros_venv_entrypoint.sh"]
CMD ["/bin/bash"]

