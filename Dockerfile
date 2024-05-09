# Pull px4-ros-noetic image
FROM px4io/px4-dev-ros-noetic

# Update the linux environment
RUN echo "Updating environment" \
    && sudo apt update -y \
    && sudo apt upgrade -y

# Check environment
RUN echo "Checking environment" \
    && export \
    && ulimit -a

# Perform rosdep
RUN echo "Use rosdep to update dependencies" \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && rosdep update

# Install build dependencies
RUN pip install scipy \
    && pip install -U wstool

# Create a workspace
RUN echo "Creating sarax workspace" \
    && cd /home \
    && mkdir -p user/sarax_ws/src/sarax

# Clone the sarax branch from the PX4-Autopilot repository
RUN cd /home/user/sarax_ws/ \
    && git clone --recursive -b v1.13.2-sarax-sim https://github.com/SaxionMechatronics/PX4-Autopilot.git

# Initialise catkin workspace
RUN cd /home/user/sarax_ws/ \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && catkin init \
    && wstool init src

# Copy the source into workspace source directory
COPY . /home/user/sarax_ws/src/sarax/

# Install dependencies using rosdep
RUN cd /home/user/sarax_ws/ \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && rosdep install --from-paths src --ignore-src -r -y --skip-keys="python-scipy"

# Build the workspace
RUN cd /home/user/sarax_ws/ \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && catkin build

# Export worskpace and update sourcing
RUN cd /home/user/sarax_ws/ \
    && echo "export SARAX_WS=$PWD" >> ~/.bashrc \
    && echo "source \$SARAX_WS/devel/setup.bash" >> ~/.bashrc
