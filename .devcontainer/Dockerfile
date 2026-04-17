FROM ros:humble
ARG USERNAME=dorten
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install SSH client for Git operations
RUN apt-get update && apt-get install -y openssh-client && rm -rf /var/lib/apt/lists/*

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add the user to the dialout group to access serial devices
    && usermod -a -G dialout $USERNAME \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y \
    sudo \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-micro-ros-msgs \
    joystick \
    ninja-build \
    pkg-config \
    gcc \
    g++ \
    lsof \
    psmisc \
    libsystemd-dev \
    libxml2-dev \
    # needed for mocap-px4-bridge
    libboost-dev \
    libboost-system-dev \
    libboost-thread-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for drone controller
RUN pip3 install meson pymavlink pyserial

# Ensure rosdep is initialized in the image (safe if already initialized)
RUN rosdep init || true


# 3. Build MAVLink Router (The Fix)
# We must COPY the code from your Pi into the build environment
# 1. Copy the folder from your Pi into a TEMPORARY spot inside the build
COPY mavlink-router ./mavlink-router_tmp

# 2. Build it, install the tool to /usr/bin, then wipe the temp folder
RUN cd ./mavlink-router_tmp && \
    ln -sf /usr/local/bin/meson /usr/bin/meson && \
    rm -rf build && \
    meson setup build . -Dsystemdsystemunitdir=no && \
    ninja -C build && \
    ninja -C build install && \
    cd .. && \
    rm -rf ./mavlink-router_tmp

ENV SHELL=/bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
