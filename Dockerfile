# ==============================================================================
# Base stage with both ROS1 Noetic and ROS2 Foxy installed
FROM ubuntu:20.04 AS ros1-ros2-base

ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=foxy

# Install basic tools for setting up apt sources
RUN apt-get update \
 && apt-get install -y \
        curl \
        jq \
        locales \
        software-properties-common \
 && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS2 apt sources
RUN ROS_APT_SOURCE_VERSION=$(\
        curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | jq -r .tag_name) \
 && curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
 && apt-get update \
 && apt-get install -y /tmp/ros2-apt-source.deb \
 && rm /tmp/ros2-apt-source.deb \
 && rm -rf /var/lib/apt/lists/*

# Add ROS1 apt sources
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
        > /etc/apt/sources.list.d/ros1.list \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | apt-key add -

# Install both ROS1 and ROS2 base packages
RUN apt-get update \
 && apt-get install -y \
        ros-"${ROS1_DISTRO}"-ros-base \
        ros-"${ROS2_DISTRO}"-ros-base \
 && rm -rf /var/lib/apt/lists/*



# ==============================================================================
# Stage with build tools added
FROM ros1-ros2-base AS base-with-build-tools

WORKDIR /app

# Install development and build tools
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        git \
        python3-catkin-tools \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init \
 && rosdep update --include-eol-distros



# ==============================================================================
# Base stage with third-party sources from VCS, but without local sources.
# We have to separate these because we don't want them in the runtime image.
FROM base-with-build-tools AS base-with-vcs-sources

# Copy and import ROS1 VCS dependencies
COPY deps/ros1-deps.rosinstall ./deps/
RUN mkdir -p ros1/src \
 && vcs import ros1/src < deps/ros1-deps.rosinstall

# Copy and import ROS2 VCS dependencies
COPY deps/ros2-deps.rosinstall ./deps/
RUN mkdir -p ros2/src \
 && vcs import ros2/src < deps/ros2-deps.rosinstall



# ==============================================================================
# Stage for dumping rosdep requirements from all ROS packages. This is usually
# skipped unless we specifically want to regenerate the dependencies file.
FROM base-with-vcs-sources AS generate-rosdep-requirements

# Copy local source packages (we really only need package.xml files).
COPY ros1/ ros1/src/
COPY ros2/ ros2/src/

# Generate rosdep requirements list from both ROS1 and ROS2 workspaces
RUN (\
    rosdep install --from-paths ros1/src/ --ignore-src \
        --rosdistro=${ROS1_DISTRO} --simulate \
 && rosdep install --from-paths ros2/src/ --ignore-src \
        --rosdistro=${ROS2_DISTRO} --simulate \
    ) | grep 'apt-get install' \
      | sed 's/.*apt-get install //' \
      | tr ' ' '\n' \
      | grep -v '^-' \
      | sort -u > /tmp/apt-rosdep-requirements.txt

# When we build only this stage, at runtime we just print the list of
# dependencies. This gets overridden in the 'full' build.
CMD ["cat", "/tmp/apt-rosdep-requirements.txt"]



# ==============================================================================
# Intermediate stage where we install all dependencies
FROM base-with-vcs-sources AS with-deps-without-local-sources

# Install rosdep apt dependencies from pre-generated file
COPY deps/apt-rosdep-requirements.txt ./deps/

RUN apt update \
 && apt install -y $(sed 's/#.*//' deps/apt-rosdep-requirements.txt) \
 && rm -rf /var/lib/apt/lists/*

# Verify all dependencies are now installed
RUN rosdep check --from-paths ros1/src/ --ignore-src \
        --rosdistro=${ROS1_DISTRO} \
 && rosdep check --from-paths ros2/src/ --ignore-src \
        --rosdistro=${ROS2_DISTRO} \
 || (echo "ERROR: Missing ROS1 dependencies!" \
         && echo "Run: ./scripts/generate-rosdep-requirements.sh" \
         && exit 1)


# Update Python setuptools and its dependencies.
# https://github.com/pypa/setuptools/issues/4478#issuecomment-2235160778
#
# We also update to pip 21 which supports PEP 600 and allows us to install
# wheels with the manylinux_2_17 platform tag. This works around a build error
# with grpcio.
#
# TODO: Revisit this when upgrading beyond Python 3.8 (Ubuntu 20.04).
RUN python3 -m pip install --upgrade \
        'pip>=21,<22' \
        setuptools \
        importlib_metadata \
        importlib_resources \
        more_itertools \
        ordered-set \
        packaging \
        platformdirs \
        tomli \
        wheel

# Force setuptools to use stdlib distutils. This is required to be able to use
# `catkin install` with our newer setuptools, which provides its own distutils.
#
# TODO: distutils was removed from the stdlib in Python 3.12. When we migrate to
# ROS 2 / colcon / Ubuntu 24.04, we don't need this workaround anymore.
ENV SETUPTOOLS_USE_DISTUTILS=stdlib

# Fix the Cython version to work around a gevent install error.
# https://github.com/gevent/gevent/issues/2076
#
# For some reason related to isolated build environments, this can't go in the
# python3-requirements.txt. We also have to avoid upgrading to a newer pip.
RUN python3 -m pip install "Cython<3.1"

# Install Python dependencies
COPY deps/python3-requirements.txt ./deps/
RUN python3 -m pip install --ignore-installed -r deps/python3-requirements.txt



################################################################################
# Now, with all dependencies installed, we can build the ROS packages into the
# install space.
FROM with-deps-without-local-sources AS ros1-builder

# Initialize the catkin workspace and pre-build the third-party packages.
# This list can be updated according to `catkin build --dry-run phyto_arm`.
RUN bash -c "source /opt/ros/${ROS1_DISTRO}/setup.bash \
 && cd ros1 \
 && catkin config --install \
 && catkin build \
        ds_asio \
        ds_base \
        ds_core_msgs \
        ds_core_msgs \
        ds_nmea_msgs \
        ds_param \
        ds_sensor_msgs \
        ds_sensor_msgs \
        ds_util_nodes \
        ds_util_nodes \
        rtsp_camera \
        rtsp_camera \
        triton_classifier \
 "

# Build and test local packages
COPY ros1/ ros1/src/

RUN bash -c "cd ros1 \
 && source install/setup.bash \
 && stdbuf -o L catkin build phyto_arm \
 && stdbuf -o L catkin test -- \
        aml_ctd \
        dli_power_switch \
        ifcb \
        jvl_motor \
        phyto_arm \
        rbr_maestro3_ctd \
"



################################################################################
# Build ROS2 message packages and ros1_bridge
FROM with-deps-without-local-sources AS ros2-bridge-builder

# Configure ccache for faster builds
RUN apt-get update \
 && apt-get install -y ccache \
 && rm -rf /var/lib/apt/lists/*

ENV PATH="/usr/lib/ccache:${PATH}"
ENV CCACHE_DIR="/ccache"
ENV CCACHE_MAXSIZE="2G"

# Copy the ROS1 install space from ros1-builder
COPY --from=ros1-builder /app/ros1/install /app/ros1/install

# Synthesize ROS2 messages from ROS1 packages using ros2_convert_msg tool
RUN bash -c " \
    source /opt/ros/${ROS1_DISTRO}/setup.bash && \
    source /app/ros1/install/setup.bash && \
    for pkg in \$(rospack list | grep '/app/ros1/install' | awk '{print \$1}'); do \
        pkg_path=\$(rospack find \$pkg); \
        python3 /app/ros2/src/ros2_convert_msg/convert.py \$pkg_path /app/ros2/src/; \
    done \
    "

# Build the synthesized ROS2 message packages
RUN bash -c " \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    cd /app/ros2 && \
    colcon build --merge-install \
    "

# Clone ros1_bridge
RUN mkdir -p /app/bridge/src \
 && cd /app/bridge \
 && git clone https://github.com/ros2/ros1_bridge.git src/ros1_bridge \
 && (cd src/ros1_bridge && git checkout "${ROS2_DISTRO}" || true)

# Fix xmlrpcpp dependency issue (https://github.com/ros2/ros1_bridge/issues/459)
RUN sed -i '/xmlrpcpp/d' /app/bridge/src/ros1_bridge/package.xml

# Add std_msgs converters (https://github.com/ros2/ros1_bridge/issues/464)
COPY patches/std_msgs_converters.cpp /app/bridge/src/ros1_bridge/src/
RUN sed -i \
    '/"src\/builtin_interfaces_factories.cpp"/a "src/std_msgs_converters.cpp"' \
    /app/bridge/src/ros1_bridge/CMakeLists.txt

# Install ros1_bridge dependencies
RUN bash -c " \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    rosdep install -y \
        --from-paths /app/bridge/src \
        --ignore-src \
        --skip-keys 'fastcdr rti-connext-dds-6.0.1 urdfdom_headers demo_nodes_cpp' \
        --rosdistro '${ROS2_DISTRO}' \
    "

# Build ros1_bridge with all environments sourced
RUN --mount=type=cache,target=/ccache \
    bash -c " \
    source /opt/ros/${ROS1_DISTRO}/setup.bash && \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    source /app/ros1/install/setup.bash && \
    source /app/ros2/install/local_setup.bash && \
    cd /app/bridge && \
    MAKEFLAGS=-j1 colcon build \
        --packages-select ros1_bridge \
        --cmake-force-configure \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers console_cohesion+ \
    "



################################################################################
# Finally, we create a runtime stage with only installed packages
FROM with-deps-without-local-sources AS runtime


# Install the ROS Launchpad management server
RUN mkdir -p /launchpad \
 && curl -L http://github.com/WHOIGit/ros-launchpad/archive/v1.0.14.tar.gz \
        | tar zxf - --strip-components=1 -C /launchpad \
 && python3 -m pip install --ignore-installed -r /launchpad/requirements.txt


# Install the launch tools and server files
COPY ./phyto-arm ./phyto-arm


# Create hot-reload workspace directory structure
RUN mkdir -p /hot/ros1/src


# Expose web interface port
EXPOSE 8080


# Source ROS environment automatically for all bash sessions
RUN echo 'source /app/ros1/install/setup.bash' >> /etc/bash.bashrc \
 && echo 'if [ -f /hot/ros1/devel/setup.bash ]; then source /hot/ros1/devel/setup.bash; fi' >> /etc/bash.bashrc

# Install the entrypoint script.
# The entrypoint path is inherited from the ROS base image.
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Default command runs the server with ROS environment sourced
CMD ["/bin/bash", "-c", "cd /launchpad && python3 server.py --package phyto_arm --config /app/mounted_config.yaml /app/configs/example.yaml"]


# Finally, copy the ROS install spaces from both builders
COPY --from=ros1-builder /app/ros1/install /app/ros1/install
COPY --from=ros2-bridge-builder /app/ros2/install /app/ros2/install
COPY --from=ros2-bridge-builder /app/bridge/install /app/bridge/install
