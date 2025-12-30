# ==============================================================================
# Base stage without any sources
FROM ros:noetic AS base

WORKDIR /app

# Base dependencies required to bootstrap a build environment. Package-specific
# dependencies should be added to package.xml and resolved with rosdep.
RUN apt update && apt install -y \
    build-essential \
    git \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep update --rosdistro=${ROS_DISTRO}



# ==============================================================================
# Base stage with third-party sources from VCS, but without local sources.
# We have to separate these because we don't want them in the runtime image.
FROM base AS base-with-vcs-sources

# Copy and import VCS dependencies
COPY deps/deps.rosinstall ./deps/
RUN mkdir -p src \
 && vcs import src < deps/deps.rosinstall



# ==============================================================================
# Stage for dumping rosdep requirements from all ROS packages. This is usually
# skipped unless we specifically want to regenerate the dependencies file.
FROM base-with-vcs-sources AS generate-rosdep-requirements

# Copy local source packages (we really only need package.xml files).
COPY src/ src/

# Generate rosdep requirements list
RUN rosdep install --from-paths src/ --ignore-src \
        --rosdistro=${ROS_DISTRO} --simulate \
        | grep 'apt-get install' \
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
RUN rosdep check --from-paths src/ --ignore-src || \
    (echo "ERROR: Missing dependencies!" \
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
FROM with-deps-without-local-sources AS builder

# Initialize the catkin workspace and pre-build the third-party packages.
# This list can be updated according to `catkin build --dry-run phyto_arm`.
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
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
COPY src/ src/

RUN bash -c "source install/setup.bash \
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
# Finally, we create a runtime stage with only installed packages
FROM with-deps-without-local-sources AS runtime


# Install the ROS Launchpad management server
RUN mkdir -p /launchpad \
 && curl -L http://github.com/WHOIGit/ros-launchpad/archive/v1.0.14.tar.gz \
        | tar zxf - --strip-components=1 -C /launchpad \
 && python3 -m pip install --ignore-installed -r /launchpad/requirements.txt


# Install the launch tools and server files
COPY ./phyto-arm ./phyto-arm


# Expose web interface port
EXPOSE 8080


# Source ROS environment automatically for all bash sessions
RUN echo 'source /app/install/setup.bash' >> /etc/bash.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /app/install/setup.bash && exec \"$@\"", "--"]

# Default command runs the server with ROS environment sourced
CMD ["/bin/bash", "-c", "cd /launchpad && python3 server.py --package phyto_arm --config /app/mounted_config.yaml /app/configs/example.yaml"]


# Finally, copy the ROS install space from builder
COPY --from=builder /app/install /app/install
