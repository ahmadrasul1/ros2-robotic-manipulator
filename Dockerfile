FROM docker.io/ros:jazzy

ENV SHELL=/bin/bash \
    DEBIAN_FRONTEND=noninteractive

# Install ROS, build, and Python environment dependencies.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pip \
      python3-venv \
      python3-yaml \
      python3-numpy \
      python3-matplotlib \
      git \
      cmake \
      build-essential \
      libserial-dev \
      curl \
      less \
      htop \
      tree \
      nano \
      vim \
      neovim \
      ros-jazzy-rviz2 \
      ros-jazzy-rqt-common-plugins \
      ros-jazzy-xacro \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Keep pip-installed packages separate from Debian-managed Python packages.
# --system-site-packages lets the environment continue to use ROS Python modules
# installed through apt, while pip packages are installed only inside /opt/venv.
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv --system-site-packages "${VIRTUAL_ENV}" && \
    "${VIRTUAL_ENV}/bin/python" -m pip install --no-cache-dir --upgrade \
      pip setuptools wheel && \
    "${VIRTUAL_ENV}/bin/python" -m pip install --no-cache-dir \
      pysoem==1.1.12 \
      roboticstoolbox-python==1.3.1

ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

# Clone and install SOEM.
RUN git clone --depth 1 https://github.com/OpenEtherCATsociety/SOEM.git /opt/SOEM && \
    cmake -S /opt/SOEM -B /opt/SOEM/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /opt/SOEM/build --parallel "$(nproc)" && \
    cmake --install /opt/SOEM/build && \
    ldconfig

# Set up the interactive ROS shell.
RUN printf '%s\n' \
      'echo "rosbuild - Build all packages"' \
      'echo "rossetup - Source ROS local setup variable"' \
      'echo "rosclean - Delete build, install and log"' \
      "alias rossetup='cd /root/ws && source /root/ws/install/local_setup.bash && ros2 daemon start'" \
      "alias rosbuild='colcon build --symlink-install'" \
      "alias rosclean='rm -rf /root/ws/build /root/ws/install /root/ws/log'" \
      'source /opt/ros/jazzy/setup.bash' \
      "PS1='\\[\\e[32m\\]rascl-container\\[\\e[0m\\]:\\[\\e[34m\\]\\w\\[\\e[0m\\]$ '" \
      >> /root/.bashrc

WORKDIR /root/ws

CMD ["/bin/bash"]