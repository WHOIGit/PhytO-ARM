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

# Initialize rosdep (skip if already initialized by base image)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi \
 && rosdep update



# ==============================================================================
# Stage for dumping rosdep requirements from all ROS packages. This is usually
# skipped unless we specifically want to regenerate the dependencies file.
FROM base-with-build-tools AS generate-rosdep-requirements

# Copy local source packages (we really only need package.xml files).
COPY ros2/ ros2/src/

# Import external ROS2 dependencies from rosinstall file
COPY deps/ros2-deps.rosinstall ./deps/
RUN cd ros2/src && vcs import < /app/deps/ros2-deps.rosinstall

# Generate rosdep requirements list from ROS2 workspace
RUN rosdep install --from-paths ros2/src/ --ignore-src \
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
FROM base-with-build-tools AS with-deps

# Install rosdep apt dependencies from pre-generated file
COPY deps/apt-rosdep-requirements.txt ./deps/

# Install apt dependencies
RUN apt update \
 && apt install -y $(sed 's/#.*//' deps/apt-rosdep-requirements.txt) \
 && rm -rf /var/lib/apt/lists/*

# Verify all dependencies are now installed
RUN rosdep check --from-paths ros2/src/ --ignore-src \
        --rosdistro=${ROS_DISTRO} \
 || (echo "ERROR: Missing rosdep dependencies!" \
         && echo "Run: ./scripts/generate-rosdep-requirements.sh" \
         && exit 1)

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
# Build ROS2 packages
FROM with-deps AS ros2-builder

# Copy all local ROS2 source packages
COPY ros2/ ros2/src/

# Import external ROS2 dependencies from rosinstall file
COPY deps/ros2-deps.rosinstall ./deps/
RUN cd ros2/src && vcs import < /app/deps/ros2-deps.rosinstall

# Build all ROS2 packages with colcon.
#
# We use a symlinked install space so that we can mount over some files without
# requiring a rebuild of the workspace. This is best-effort; you
RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /app/ros2 && \
    colcon build --merge-install --symlink-install --packages-up-to phyto_arm \
    "



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

# Copy the ROS2 workspace from builder.
# src and build subdirectories are only needed for a symlinked install space
COPY --from=ros2-builder /app/ros2/install /app/ros2/install
COPY --from=ros2-builder /app/ros2/build /app/ros2/build
COPY --from=ros2-builder /app/ros2/src /app/ros2/src
