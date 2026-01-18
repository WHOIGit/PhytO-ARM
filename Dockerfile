# ==============================================================================
# Base image used by both the build and runtime stages
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS ros2-base

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
        python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi \
 && rosdep update --rosdistro="${ROS_DISTRO}" --include-eol-distros



# ==============================================================================
# Share one checkout of the pinned native ports and compatibility layer.
FROM base-with-build-tools AS external-sources
COPY deps/ros2-deps.rosinstall ./deps/
RUN mkdir -p ros2/src \
 && cd ros2/src \
 && vcs import < /app/deps/ros2-deps.rosinstall

# ==============================================================================
# Resolve the build and runtime dependencies for this ROS distribution.
FROM external-sources AS generate-rosdep-requirements
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Copy local source packages (we really only need package.xml files).
COPY ros2/ ros2/src/

# Generate rosdep apt requirements (apt:<package>)
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && cd ros2 \
 && colcon list --packages-up-to phyto_arm --paths-only > /tmp/package-paths.txt \
 && rosdep install --from-paths $(cat /tmp/package-paths.txt) --ignore-src \
        --rosdistro="${ROS_DISTRO}" --simulate > /tmp/rosdep-install.txt \
 && sed -n 's/^ *apt-get install //p' /tmp/rosdep-install.txt \
    | tr ' ' '\n' | grep -v '^-' | sort -u | sed 's/^/apt:/' > /tmp/apt-packages.txt

CMD ["sed", "s/^apt://", "/tmp/apt-packages.txt"]


# ==============================================================================
# Intermediate stage where we install all dependencies
FROM base-with-build-tools AS with-deps

# Install native extensions with the distro Python and keep the remaining
# Python dependencies outside the distribution's package-managed environment.
COPY --from=generate-rosdep-requirements /tmp/apt-packages.txt /tmp/apt-packages.txt
RUN apt-get update \
 && apt-get install -y $(sed 's/^apt://' /tmp/apt-packages.txt) \
 && rm -rf /var/lib/apt/lists/*

COPY deps/python3-requirements.txt ./deps/
RUN python3 -m pip install --no-cache-dir --target=/opt/phyto-python -r deps/python3-requirements.txt
ENV PYTHONPATH=/opt/phyto-python

################################################################################
FROM with-deps AS ros2-builder
COPY --from=external-sources /app/ros2/src /app/ros2/src
COPY ros2/ ros2/src/
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd /app/ros2 && colcon build --merge-install --symlink-install --packages-up-to phyto_arm"

################################################################################
# Finally, we create a runtime stage with only installed packages
FROM with-deps AS runtime

# Install the launch tools and server files
COPY ./phyto-arm ./phyto-arm
COPY configs/example.yaml ./configs/example.yaml

# Expose web interface port
EXPOSE 8080

# Source ROS2 environment automatically for all bash sessions
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc \
 && echo 'source /app/ros2/install/setup.bash' >> /etc/bash.bashrc

# Install the entrypoint script.
# Also provide a trampoline so you can do `docker exec ... ros2 ...`
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN ln -s /ros_entrypoint.sh /usr/local/bin/ros2

# Default command - start bash
CMD ["/bin/bash"]

# Copy the ROS2 workspace from the final build stage.
# src and build subdirectories are only needed for a symlinked install space
COPY --from=ros2-builder /app/ros2/install /app/ros2/install
COPY --from=ros2-builder /app/ros2/build /app/ros2/build
COPY --from=ros2-builder /app/ros2/src /app/ros2/src
