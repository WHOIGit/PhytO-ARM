FROM ros:noetic

ENV ROS2_DISTRO=foxy
RUN apt-get update && apt-get install -y curl \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros2-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y ros-${ROS2_DISTRO}-ros-base python3-colcon-common-extensions \
 && rosdep update --include-eol-distros \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app


# Install apt package dependencies
COPY deps/apt-requirements.txt ./
RUN apt update \
 && sed '/^#/d' apt-requirements.txt | xargs apt install -y \
 && rm -rf /var/lib/apt/lists/*

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

# Fix the Cython version to work around a gevent install error.
# https://github.com/gevent/gevent/issues/2076
#
# For some reason related to isolated build environments, this can't go in the
# python3-requirements.txt. We also have to avoid upgrading to a newer pip.
RUN python3 -m pip install "Cython<3.1"

# Install Python dependencies
COPY deps/python3-requirements.txt ./
RUN python3 -m pip install --ignore-installed -r python3-requirements.txt


# Clone third-party dependencies from VCS
WORKDIR /app/ros1
COPY deps/ros1-deps.rosinstall ./
RUN echo Installing ROS dependencies \
 && mkdir ./src \
 && vcs import src < ros1-deps.rosinstall

# Install dependencies declared in package.xml files
RUN apt update \
 && rosdep install --default-yes --from-paths ./src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

# Warm the build directory with pre-built packages that don't change often.
# This list can be updated according to `catkin build --dry-run phyto_arm`.
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
 && stdbuf -o L catkin build \
        ds_core_msgs \
        ds_sensor_msgs \
 "

# Copy package.xml files for local packages
COPY ./ros1/aml_ctd/package.xml ./src/aml_ctd/package.xml
COPY ./ros1/dli_power_switch/package.xml ./src/dli_power_switch/package.xml
COPY ./ros1/ifcb/package.xml ./src/ifcb/package.xml
COPY ./ros1/jvl_motor/package.xml ./src/jvl_motor/package.xml
COPY ./ros1/phyto_arm/package.xml ./src/phyto_arm/package.xml
COPY ./src/phyto_sim_utils/package.xml ./src/phyto_sim_utils/package.xml
COPY ./ros1/rbr_maestro3_ctd/package.xml ./src/rbr_maestro3_ctd/package.xml

# Install new rosdep dependencies declared in the above package.xml files
RUN apt update \
 && rosdep install --default-yes --from-paths ./src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

# Copy the rest of the source
COPY ./src ./src
COPY ./ros1 ./src

# Build
RUN bash -c "source devel/setup.bash \
 && stdbuf -o L catkin build phyto_arm \
 && stdbuf -o L catkin test -- \
        aml_ctd \
        dli_power_switch \
        ifcb \
        jvl_motor \
        phyto_arm \
        rbr_maestro3_ctd \
"

# Generate ROS 2 interfaces for the packages in the Noetic workspace.
WORKDIR /app/ros2
COPY deps/ros2-deps.rosinstall /app/deps/
RUN mkdir -p src && vcs import src < /app/deps/ros2-deps.rosinstall
COPY ros2/ src/
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN source /app/ros1/devel/setup.bash \
 && mkdir -p /app/bridge_interfaces/src \
 && rospack list | while read -r name path; do \
        case "$path" in /app/ros1/src/*) \
            python3 /app/ros2/src/ros2_convert_msg/convert.py "$path" /app/bridge_interfaces ;; \
        esac; \
    done
WORKDIR /app/bridge_interfaces
RUN source /opt/ros/${ROS2_DISTRO}/setup.bash \
 && apt-get update \
 && rosdep install -y --from-paths src --ignore-src --rosdistro=${ROS2_DISTRO} \
 && colcon build --merge-install \
 && rm -rf /var/lib/apt/lists/*

# Build the Foxy bridge after sourcing the interfaces from both workspaces.
WORKDIR /app/bridge
RUN git clone --depth 1 --branch ${ROS2_DISTRO} https://github.com/ros2/ros1_bridge.git src/ros1_bridge
COPY patches/std_msgs_converters.cpp src/ros1_bridge/src/
RUN sed -i '/"src\/builtin_interfaces_factories.cpp"/a\  "src/std_msgs_converters.cpp"' src/ros1_bridge/CMakeLists.txt \
 && sed -i '/xmlrpcpp/d' src/ros1_bridge/package.xml \
 && source /app/ros1/devel/setup.bash \
 && source /opt/ros/${ROS2_DISTRO}/local_setup.bash \
 && source /app/bridge_interfaces/install/local_setup.bash \
 && apt-get update \
 && rosdep install -y --from-paths src --ignore-src --rosdistro=${ROS2_DISTRO} \
        --skip-keys="fastcdr rti-connext-dds-6.0.1 urdfdom_headers demo_nodes_cpp" \
 && MAKEFLAGS=-j1 colcon build --packages-select ros1_bridge --cmake-force-configure \
 && rm -rf /var/lib/apt/lists/*

# Copy the launch tools and server files
WORKDIR /app
COPY ./phyto-arm ./phyto-arm
COPY configs/example.yaml ./configs/example.yaml

# Expose web interface port
EXPOSE 8080

# Source ROS environment automatically for all bash sessions
RUN echo "source /app/ros1/devel/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /app/ros1/devel/setup.bash && exec \"$@\"", "--"]

CMD ["/bin/bash"]
