# ==============================================================================
# Base stage with ROS2 Humble
FROM ros:humble AS ros2-base

ENV ROS2_DISTRO=humble

WORKDIR /app

# Install basic tools
RUN apt-get update \
 && apt-get install -y \
        curl \
        jq \
        locales \
 && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8



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
        --rosdistro=${ROS2_DISTRO} --simulate \
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
COPY deps/apt-requirements.txt ./deps/
RUN apt update \
 && apt install -y $(sed 's/#.*//' deps/apt-rosdep-requirements.txt || true) \
 && apt install -y $(sed 's/#.*//' deps/apt-requirements.txt || true) \
 && rm -rf /var/lib/apt/lists/*

# Update Python setuptools and its dependencies.
# https://github.com/pypa/setuptools/issues/4478#issuecomment-2235160778
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

# Build all ROS2 packages with colcon
# Note: Don't use --symlink-install in Docker as it creates broken symlinks
# when copying the install space to the runtime stage
RUN bash -c " \
    source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    cd /app/ros2 && \
    colcon build --merge-install \
    "



################################################################################
# Finally, we create a runtime stage with only installed packages
FROM with-deps AS runtime

# Install the launch tools and server files
COPY ./phyto-arm ./phyto-arm

# Create hot-reload workspace directory structure
RUN mkdir -p /hot/ros2/src

# Expose web interface port
EXPOSE 8080

# Source ROS2 environment automatically for all bash sessions
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
 && echo 'source /app/ros2/install/setup.bash 2>/dev/null || true' >> /etc/bash.bashrc \
 && echo 'if [ -f /hot/ros2/install/setup.bash ]; then source /hot/ros2/install/setup.bash; fi' >> /etc/bash.bashrc

# Install the entrypoint script.
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Default command - start bash
CMD ["/bin/bash"]

# Copy the ROS2 install space from builder
COPY --from=ros2-builder /app/ros2/install /app/ros2/install
