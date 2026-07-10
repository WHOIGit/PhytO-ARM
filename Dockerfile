# Build our own slimmed-down version of ros:noetic
FROM ubuntu:20.04 AS ros-base

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

# Only install packages we explicitly request
RUN echo 'APT::Install-Recommends "false";' > /etc/apt/apt.conf.d/99-no-recommends

# ROS Noetic reached end of life in May 2025, so packages are installed from
# the final snapshot of the package archive.
RUN echo 'Etc/UTC' > /etc/timezone \
 && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        tzdata \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir -p /usr/share/keyrings \
 && curl -fsSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA' \
        | gpg --dearmor > /usr/share/keyrings/ros1-snapshots-archive-keyring.gpg \
 && rm -rf /root/.gnupg \
 && echo "deb [ signed-by=/usr/share/keyrings/ros1-snapshots-archive-keyring.gpg ] http://snapshots.ros.org/noetic/final/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros1-snapshots.list

# OpenMPI, a ROS dependency by way of Boost, requires a Fortran compiler, so
# apt pulls in the LLVM Fortran compiler, which is massive. Block it so apt
# selects GFortran instead (-1.11 GB).
RUN printf 'Package: flang-18 libflang-18-dev\nPin: release *\nPin-Priority: -1\n' \
        > /etc/apt/preferences.d/no-flang

# Install a shim package that satisfies dependencies on -dev packages that
# aren't really needed at runtime. The builder stage removes the shim and
# installs the real packages. To maintain this list, find any -dev packages
# in the final image and use 'apt-cache rdepends --installed' to see what
# requires them.
RUN mkdir -p /tmp/shim/DEBIAN \
 && provides=$(echo \
        cmake \
        google-mock \
        libapr1-dev \
        libaprutil1-dev \
        libboost-all-dev \
        libboost-chrono-dev \
        libboost-date-time-dev \
        libboost-dev \
        libboost-filesystem-dev \
        libboost-program-options-dev \
        libboost-regex-dev \
        libboost-system-dev \
        libboost-thread-dev \
        libbz2-dev \
        libconsole-bridge-dev \
        libgpgme-dev \
        libgtest-dev \
        liblog4cxx-dev \
        liblz4-dev \
        libopencv-dev \
        libpoco-dev \
        libssl-dev \
        libtinyxml2-dev \
        libturbojpeg0-dev \
        python3-dev \
        uuid-dev \
    | sed 's/ /, /g') \
 && printf '%s\n' \
        'Package: dev-dependency-shim' \
        'Version: 1.0' \
        'Architecture: all' \
        'Maintainer: PhytO-ARM developers' \
        "Provides: $provides" \
        'Description: Stand-in for build-time dependencies of ROS packages' \
        > /tmp/shim/DEBIAN/control \
 && dpkg-deb --build /tmp/shim /tmp/shim.deb \
 && dpkg -i /tmp/shim.deb \
 && rm -rf /tmp/shim /tmp/shim.deb

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        python3-rosdep \
        ros-noetic-ros-base=1.5.0-1* \
 && rm -rf /var/lib/apt/lists/* \
 && rosdep init \
 && rosdep update --rosdistro $ROS_DISTRO

WORKDIR /app


# Use an intermediate builder stage to compile dependencies
FROM ros-base AS builder

# The shim hides development packages that the builder needs, so remove it
# and let apt install the real packages.
RUN dpkg --remove --force-depends dev-dependency-shim \
 && apt-get update \
 && apt-get install -y --fix-broken --no-install-recommends \
 && rm -rf /var/lib/apt/lists/*


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
COPY deps/deps.rosinstall ./
RUN echo Installing ROS dependencies \
 && mkdir ./src \
 && vcs import src < deps.rosinstall

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
        ds_util_nodes \
        rtsp_camera \
 "

# Copy package.xml files for local packages
COPY ./src/aml_ctd/package.xml ./src/aml_ctd/package.xml
COPY ./src/dli_power_switch/package.xml ./src/dli_power_switch/package.xml
COPY ./src/ifcb/package.xml ./src/ifcb/package.xml
COPY ./src/jvl_motor/package.xml ./src/jvl_motor/package.xml
COPY ./src/phyto_arm/package.xml ./src/phyto_arm/package.xml
COPY ./src/rbr_maestro3_ctd/package.xml ./src/rbr_maestro3_ctd/package.xml

# Install new rosdep dependencies declared in the above package.xml files
RUN apt update \
 && rosdep install --default-yes --from-paths ./src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

# Copy the rest of the source
COPY ./src ./src

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

# Clone the ROS Launchpad management server
RUN mkdir -p /launchpad
RUN curl -L http://github.com/WHOIGit/ros-launchpad/archive/v1.0.14.tar.gz | tar zxf - --strip-components=1 -C /launchpad
RUN python3 -m pip install --ignore-installed -r /launchpad/requirements.txt


# Final build stage, leaving behind build-time dependencies
FROM ros-base

# Install only the dependencies needed to run the workspace
COPY --from=builder /app/src ./src
RUN apt update \
 && rosdep install --default-yes --dependency-types exec --from-paths ./src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

# Copy the built workspace, Python packages installed by pip, and the ROS
# Launchpad management server
COPY --from=builder /app/devel ./devel
COPY --from=builder /usr/local /usr/local
COPY --from=builder /launchpad /launchpad

# Copy the launch tools and server files
COPY ./phyto-arm ./phyto-arm

# Expose web interface port
EXPOSE 8080

# Source ROS environment automatically for all bash sessions
RUN echo "source /app/devel/setup.bash" >> /etc/bash.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /app/devel/setup.bash && exec \"$@\"", "--"]

# Default command runs the server with ROS environment sourced
CMD ["/bin/bash", "-c", "cd /launchpad && python3 server.py --package phyto_arm --config /app/mounted_config.yaml /app/configs/example.yaml"]
