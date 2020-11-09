FROM ros:melodic

# Install the rosexec wrapper
COPY rosexec /usr/bin/

# Create an empty catkin workspace
WORKDIR /root
RUN mkdir ./src \
    && rosexec catkin_make

# Install third-party dependencies
COPY ./deps.rosinstall ./
RUN wstool init --shallow src deps.rosinstall

# Copy package.xml file only
COPY ./src/pa_base/package.xml ./src/pa_base/package.xml

# Install dependencies
RUN apt-get update \
    && rosdep install --default-yes --from-paths ./src --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# Copy the rest of the sources
COPY ./src ./src

# Build
RUN rosexec catkin_make

CMD rosexec roslaunch --wait pa_base phyto_arm.launch
