FROM ubuntu:20.04 AS base

RUN apt-get update && apt-get install -y    \
    curl                                    \
    locales                                 \
    software-properties-common              \
    && rm -rf /var/lib/apt/lists/*

# Set the locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install ROS 2
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install -y ros-foxy-ros-base && rm -rf /var/lib/apt/lists/*

# ==========================================================================================================
# ==========================================================================================================

FROM base AS build

RUN apt update && apt install -y ros-dev-tools python3-pip && rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update --rosdistro foxy

# Build
SHELL [ "/bin/bash", "-c" ]
WORKDIR /workspace
COPY src ./src
RUN apt update && rosdep install -i --from-path src --rosdistro foxy -r -y && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/foxy/setup.bash && colcon build --merge-install
RUN rm -rf build log

# install joy_node
RUN apt update && apt install -y ros-foxy-joy* && rm -rf /var/lib/apt/lists/*

# ==========================================================================================================
# ==========================================================================================================

FROM build AS runtime

RUN apt update && apt install -y supervisor && rm -rf /var/lib/apt/lists/*

COPY runtime/supervisord.conf /etc/supervisord.conf
COPY runtime/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]