FROM ros:noetic
# Increment DEPSCACHE when there's a known change to deps.rosinstall
ARG DEPSCACHE=1
SHELL ["/usr/bin/bash", "-c"]
WORKDIR /app

# Install apt package dependencies
COPY deps/apt-requirements.txt ./
RUN apt update \
 && sed '/^#/d' apt-requirements.txt | xargs apt install -y \
 && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY deps/python3-requirements.txt ./
RUN pip install --upgrade setuptools==69.1.0
RUN python3 -m pip install -r python3-requirements.txt


# Clone third-party dependencies from VCS
COPY deps/deps.rosinstall ./
RUN echo Installing ROS dependencies:${DEPSCACHE} \
 && mkdir ./src \
 && vcs import src < deps.rosinstall

# Install dependencies declared in package.xml files
RUN apt update \
 && rosdep install --default-yes --from-paths ./src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

# Warm the build directory with pre-built packages that don't change often.
# This list can be updated according to `catkin build --dry-run phyto_arm`.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && stdbuf -o L catkin build \
        ds_core_msgs \
        ds_sensor_msgs \
        ds_util_nodes \
        rtsp_camera

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
RUN source ./devel/setup.bash \
 && stdbuf -o L catkin build phyto_arm

# Copy the launch tool
ENV DONT_SCREEN=1
ENV NO_VIRTUALENV=1
COPY ./phyto-arm ./phyto-arm
