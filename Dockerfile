# ==============================================================================
# Base stage with ROS2 Humble
FROM ros:humble AS ros2-base

WORKDIR /app


# ==============================================================================
# Stage with build tools added
FROM ros2-base AS base-with-build-tools

# Install development and build tools
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        git \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi \
 && rosdep update



# ==============================================================================
# Stage for generating dependency lists. This is usually skipped unless we
# specifically want to regenerate the dependencies files.
FROM base-with-build-tools AS generate-dependencies

# Import external ROS2 dependencies from rosinstall file
COPY deps/ros2-deps.rosinstall ./deps/
RUN mkdir -p ros2/src \
 && cd ros2/src \
 && vcs import < /app/deps/ros2-deps.rosinstall

# Generate list of external packages defined in rosinstall (for filtering)
RUN cd ros2 \
 && colcon list --names-only --base-paths src \
        > /tmp/external-packages.txt

# Copy local source packages (we really only need package.xml files).
COPY ros2/ ros2/src/

# Generate rosdep apt requirements (apt:<package>)
RUN rosdep install --from-paths ros2/src/ --ignore-src \
        --rosdistro=${ROS_DISTRO} --simulate \
    | grep 'apt-get install' \
    | sed 's/.*apt-get install //' \
    | tr ' ' '\n' \
    | grep -v '^-' \
    | sort -u \
    | sed 's/^/apt:/' > /tmp/apt-packages.txt

# Generate package info (pkg:<name>\t<path>\t<is_external>)
# Path is relative to ros2/src/ for local packages, empty for external
RUN cd ros2 \
 && colcon list --packages-up-to phyto_arm --topological-order \
    | awk -F'\t' '\
        NR==FNR { ext[$1]; next } \
        { \
            name = $1; \
            path = $2; \
            sub(/^src\//, "", path); \
            is_ext = (name in ext) ? "true" : "false"; \
            if (is_ext == "true") path = ""; \
            print "pkg:" name "\t" path "\t" is_ext \
        }' /tmp/external-packages.txt - > /tmp/package-info.txt

# Generate dependency graph (dep:<package>\t<dependency>)
# Each line represents: <package> depends on <dependency>
RUN cd ros2 \
 && colcon graph --dot --packages-up-to phyto_arm 2>/dev/null \
    | grep -- '->' \
    | sed 's/^[[:space:]]*"\([^"]*\)"[[:space:]]*->[[:space:]]*"\([^"]*\)".*/dep:\1\t\2/' \
    > /tmp/deps.txt || true

# Output format (one section per line prefix):
# apt:<package>
# pkg:<name>\t<path>\t<is_external>
# dep:<package>\t<dependency>
CMD ["sh", "-c", "cat /tmp/apt-packages.txt /tmp/package-info.txt /tmp/deps.txt"]



# ==============================================================================
# Intermediate stage where we install all dependencies
FROM base-with-build-tools AS with-deps

# Install apt dependencies
COPY deps/apt-rosdep-requirements.txt ./deps/
RUN apt update \
 && apt install -y $(sed 's/#.*//' deps/apt-rosdep-requirements.txt) \
 && rm -rf /var/lib/apt/lists/*

# Update Python setuptools and its dependencies.
# https://github.com/pypa/setuptools/issues/4478#issuecomment-2235160778
#
# We also update to pip 21 which supports PEP 600 and allows us to install
# wheels with the manylinux_2_17 platform tag. This works around a build error
# with grpcio.
#
# We pin setuptools<80 because newer versions break editable installs in
# symlinked install spaces.
# https://github.com/pypa/setuptools/issues/4971
#
# TODO: Revisit this when upgrading beyond Python 3.8 (Ubuntu 20.04).
RUN python3 -m pip install --upgrade \
        'pip>=21,<22' \
        'setuptools<80' \
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
COPY deps/python3-requirements.txt ./deps/
RUN python3 -m pip install --ignore-installed -r deps/python3-requirements.txt



################################################################################
# Stage with external package sources imported
FROM with-deps AS with-sources

# Create workspace and import external ROS2 dependencies
COPY deps/ros2-deps.rosinstall ./deps/
RUN mkdir -p ros2/src \
 && cd ros2 && colcon build \
 && cd src && vcs import < /app/deps/ros2-deps.rosinstall


################################################################################
# Build packages in separate stages because colcon is super slow.
# This allows BuildKit to handle parallelism and layer caching.
#
# This section is generated by scripts/generate-build-stages.sh
#
# BEGIN GENERATED BUILD STEPS
FROM with-sources AS build-ds_core_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select ds_core_msgs"

FROM with-sources AS build-ds_nmea_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select ds_nmea_msgs"

FROM with-sources AS build-ifcb_msgs
COPY ros2/ifcb_msgs ros2/src/ifcb_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select ifcb_msgs"

FROM with-sources AS build-phyto_arm_msgs
COPY ros2/phyto_arm_msgs ros2/src/phyto_arm_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select phyto_arm_msgs"

FROM with-sources AS build-rospy_too
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select rospy_too"

FROM with-sources AS build-wr2_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select wr2_msgs"

FROM with-sources AS build-aml_ctd_msgs
COPY ros2/aml_ctd_msgs ros2/src/aml_ctd_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select aml_ctd_msgs"

FROM with-sources AS build-dli_power_switch_msgs
COPY ros2/dli_power_switch_msgs ros2/src/dli_power_switch_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select dli_power_switch_msgs"

FROM with-sources AS build-ds_sensor_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-ds_nmea_msgs /app/ros2/install/ds_nmea_msgs /app/ros2/install/ds_nmea_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select ds_sensor_msgs"

FROM with-sources AS build-ifcb
COPY ros2/ifcb ros2/src/ifcb
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-ifcb_msgs /app/ros2/install/ifcb_msgs /app/ros2/install/ifcb_msgs
COPY --from=build-rospy_too /app/ros2/install/rospy_too /app/ros2/install/rospy_too
COPY --from=build-wr2_msgs /app/ros2/install/wr2_msgs /app/ros2/install/wr2_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select ifcb"

FROM with-sources AS build-jvl_motor_msgs
COPY ros2/jvl_motor_msgs ros2/src/jvl_motor_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select jvl_motor_msgs"

FROM with-sources AS build-rbr_maestro3_ctd_msgs
COPY ros2/rbr_maestro3_ctd_msgs ros2/src/rbr_maestro3_ctd_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select rbr_maestro3_ctd_msgs"

FROM with-sources AS build-wr2_base
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-wr2_msgs /app/ros2/install/wr2_msgs /app/ros2/install/wr2_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select wr2_base"

FROM with-sources AS build-aml_ctd
COPY ros2/aml_ctd ros2/src/aml_ctd
COPY --from=build-aml_ctd_msgs /app/ros2/install/aml_ctd_msgs /app/ros2/install/aml_ctd_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-ds_nmea_msgs /app/ros2/install/ds_nmea_msgs /app/ros2/install/ds_nmea_msgs
COPY --from=build-ds_sensor_msgs /app/ros2/install/ds_sensor_msgs /app/ros2/install/ds_sensor_msgs
COPY --from=build-rospy_too /app/ros2/install/rospy_too /app/ros2/install/rospy_too
COPY --from=build-wr2_msgs /app/ros2/install/wr2_msgs /app/ros2/install/wr2_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select aml_ctd"

FROM with-sources AS build-dli_power_switch
COPY ros2/dli_power_switch ros2/src/dli_power_switch
COPY --from=build-dli_power_switch_msgs /app/ros2/install/dli_power_switch_msgs /app/ros2/install/dli_power_switch_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-rospy_too /app/ros2/install/rospy_too /app/ros2/install/rospy_too
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select dli_power_switch"

FROM with-sources AS build-jvl_motor
COPY ros2/jvl_motor ros2/src/jvl_motor
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-jvl_motor_msgs /app/ros2/install/jvl_motor_msgs /app/ros2/install/jvl_motor_msgs
COPY --from=build-rospy_too /app/ros2/install/rospy_too /app/ros2/install/rospy_too
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select jvl_motor"

FROM with-sources AS build-rbr_maestro3_ctd
COPY ros2/rbr_maestro3_ctd ros2/src/rbr_maestro3_ctd
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-ds_nmea_msgs /app/ros2/install/ds_nmea_msgs /app/ros2/install/ds_nmea_msgs
COPY --from=build-ds_sensor_msgs /app/ros2/install/ds_sensor_msgs /app/ros2/install/ds_sensor_msgs
COPY --from=build-rbr_maestro3_ctd_msgs /app/ros2/install/rbr_maestro3_ctd_msgs /app/ros2/install/rbr_maestro3_ctd_msgs
COPY --from=build-rospy_too /app/ros2/install/rospy_too /app/ros2/install/rospy_too
COPY --from=build-wr2_msgs /app/ros2/install/wr2_msgs /app/ros2/install/wr2_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select rbr_maestro3_ctd"

FROM with-sources AS build-wr2_asio
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-wr2_base /app/ros2/install/wr2_base /app/ros2/install/wr2_base
COPY --from=build-wr2_msgs /app/ros2/install/wr2_msgs /app/ros2/install/wr2_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select wr2_asio"

FROM with-sources AS build-phyto_arm
COPY ros2/phyto_arm ros2/src/phyto_arm
COPY --from=build-aml_ctd /app/ros2/install/aml_ctd /app/ros2/install/aml_ctd
COPY --from=build-aml_ctd_msgs /app/ros2/install/aml_ctd_msgs /app/ros2/install/aml_ctd_msgs
COPY --from=build-dli_power_switch /app/ros2/install/dli_power_switch /app/ros2/install/dli_power_switch
COPY --from=build-dli_power_switch_msgs /app/ros2/install/dli_power_switch_msgs /app/ros2/install/dli_power_switch_msgs
COPY --from=build-ds_core_msgs /app/ros2/install/ds_core_msgs /app/ros2/install/ds_core_msgs
COPY --from=build-ds_nmea_msgs /app/ros2/install/ds_nmea_msgs /app/ros2/install/ds_nmea_msgs
COPY --from=build-ds_sensor_msgs /app/ros2/install/ds_sensor_msgs /app/ros2/install/ds_sensor_msgs
COPY --from=build-ifcb /app/ros2/install/ifcb /app/ros2/install/ifcb
COPY --from=build-ifcb_msgs /app/ros2/install/ifcb_msgs /app/ros2/install/ifcb_msgs
COPY --from=build-jvl_motor /app/ros2/install/jvl_motor /app/ros2/install/jvl_motor
COPY --from=build-jvl_motor_msgs /app/ros2/install/jvl_motor_msgs /app/ros2/install/jvl_motor_msgs
COPY --from=build-phyto_arm_msgs /app/ros2/install/phyto_arm_msgs /app/ros2/install/phyto_arm_msgs
COPY --from=build-rbr_maestro3_ctd /app/ros2/install/rbr_maestro3_ctd /app/ros2/install/rbr_maestro3_ctd
COPY --from=build-rbr_maestro3_ctd_msgs /app/ros2/install/rbr_maestro3_ctd_msgs /app/ros2/install/rbr_maestro3_ctd_msgs
COPY --from=build-rospy_too /app/ros2/install/rospy_too /app/ros2/install/rospy_too
COPY --from=build-wr2_asio /app/ros2/install/wr2_asio /app/ros2/install/wr2_asio
COPY --from=build-wr2_base /app/ros2/install/wr2_base /app/ros2/install/wr2_base
COPY --from=build-wr2_msgs /app/ros2/install/wr2_msgs /app/ros2/install/wr2_msgs
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /app/ros2/install/setup.bash && \
    cd /app/ros2 && \
    colcon build --packages-select phyto_arm"

# END GENERATED BUILD STEPS



################################################################################
# Finally, we create a runtime stage with only installed packages
FROM with-deps AS runtime

# Install the launch tools and server files
COPY ./phyto-arm ./phyto-arm

# Expose web interface port
EXPOSE 8080

# Source ROS2 environment automatically for all bash sessions
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
 && echo 'source /app/ros2/install/setup.bash' >> /etc/bash.bashrc

# Install the entrypoint script.
# Also provide a trampoline so you can do `docker exec ... ros2 ...`
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN ln -s /ros_entrypoint.sh /usr/local/bin/ros2

# Default command - start bash
CMD ["/bin/bash"]

# Copy the ROS2 workspace from the final build stage.
# src and build subdirectories are only needed for a symlinked install space
COPY --from=build-phyto_arm /app/ros2/install /app/ros2/install
COPY --from=build-phyto_arm /app/ros2/build /app/ros2/build
COPY --from=build-phyto_arm /app/ros2/src /app/ros2/src
