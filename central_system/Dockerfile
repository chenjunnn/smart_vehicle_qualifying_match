FROM ros:galactic

# create workspace
RUN mkdir -p /root/ros_ws/src
WORKDIR /root/ros_ws/

# copy source code
COPY . src/central_system

# install dependencies
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install remi && \
    rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# build source
RUN . /opt/ros/galactic/setup.sh && \
    colcon build \
    --symlink-install

# run app
CMD . install/setup.sh && ros2 run central_system talk_and_listen_server
