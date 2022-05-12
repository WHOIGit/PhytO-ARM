FROM ros:noetic

SHELL ["/usr/bin/bash", "-c"]
WORKDIR /app

# Install apt package dependencies
COPY ./apt-requirements.txt ./
RUN apt update \
 && sed '/^#/d' apt-requirements.txt | xargs apt install -y \
 && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY ./python3-requirements.txt ./
RUN python3 -m pip install -r python3-requirements.txt

# Clone third-party dependencies from VCS
COPY ./deps.rosinstall ./
RUN mkdir ./src \
 && vcs import src < deps.rosinstall

# Copy package.xml files for local packages
COPY ./src/aml_ctd/package.xml ./src/aml_ctd/package.xml
COPY ./src/ifcb/package.xml ./src/ifcb/package.xml
COPY ./src/jvl_motor/package.xml ./src/jvl_motor/package.xml
COPY ./src/phyto_arm/package.xml ./src/phyto_arm/package.xml
COPY ./src/rbr_maestro3_ctd/package.xml ./src/rbr_maestro3_ctd/package.xml

# Install dependencies declared in package.xml files
RUN apt update \
 && rosdep install --default-yes --from-paths ./src --ignore-src \
 && rm -rf /var/lib/apt/lists/*

# Copy the rest of the sources
COPY ./src ./src

# Build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && stdbuf -o L catkin build phyto_arm
