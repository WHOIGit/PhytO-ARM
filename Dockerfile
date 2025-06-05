FROM ros:noetic

WORKDIR /app


# Replace expired GPG key:
# https://discourse.ros.org/t/ros-gpg-key-expiration-incident/20669
ADD https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    /tmp/open-robotics.asc
RUN rm -f /usr/share/keyrings/ros1-latest-archive-keyring.gpg \
 && gpg --dearmor -o /usr/share/keyrings/ros1-latest-archive-keyring.gpg \
        /tmp/open-robotics.asc \
 && rm -f /tmp/open-robotics.asc


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
RUN python3 -m pip install -r python3-requirements.txt


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
COPY ./src/ifcb/package.xml ./src/ifcb/package.xml
COPY ./src/jvl_motor/package.xml ./src/jvl_motor/package.xml
COPY ./src/phyto_arm/package.xml ./src/phyto_arm/package.xml
COPY ./src/rbr_maestro3_ctd/package.xml ./src/rbr_maestro3_ctd/package.xml

# Install new rosdep dependencies declared in the above package.xml files
RUN apt update \
 && rosdep install --default-yes --from-paths ./src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

# Copy the rest of the sources
COPY ./src ./src
COPY ./scripts ./scripts

# Build
RUN bash -c "source devel/setup.bash \
 && stdbuf -o L catkin build phyto_arm \
 && stdbuf -o L catkin test -- \
        aml_ctd \
        ifcb \
        jvl_motor \
        phyto_arm \
        rbr_maestro3_ctd \
"

# Copy the launch tool
ENV DONT_SCREEN=1
ENV NO_VIRTUALENV=1
COPY ./phyto-arm ./phyto-arm
