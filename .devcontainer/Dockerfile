FROM ros:noetic-robot

ARG USERNAME=user-name-goes-here
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    curl \
    wget \
    apt-utils \
    git

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN curl -s http://packages.osrfoundation.org/gazebo.key | apt-key add -

ENV DEBIAN_FRONTEND="noninteractive" 
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    libcurl4-openssl-dev \
    python \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    rviz \
    x264 \
    libx264-dev \
    libignition-math4 \
    libignition-math4-dev \
    gazebo11 \
    libgazebo11-dev \
    python3-rosdep \
    xvfb \
    x11vnc \
    jwm \
    ros-noetic-media-export \
    ros-noetic-ros-base \
    ros-noetic-rosmsg \
    ros-noetic-rqt-common-plugins \
    ros-noetic-web-video-server \
    ros-noetic-xacro \ 
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-dev

# Link AWS CPP libraries
RUN ln -s /opt/ros/noetic/lib/libaws* /usr/local/lib/ && \
    ln -s /opt/ros/noetic/include/aws /usr/local/include/ 

# Install Python PIP modules
ADD .devcontainer/requirements.txt /
RUN python3 -m pip install -U --no-cache-dir pip 'setuptools<65' && \
    export SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True && \
    pip3 install -U --no-cache-dir -r requirements.txt "tensorflow-cpu==2.13.1" --ignore-installed psutil && \
    pip3 install -U --no-cache-dir "setuptools<46"

# Patch RL Coach
ADD bundle/rl_coach.patch /tmp/
RUN patch -p1 -N --directory=/usr/local/lib/python3.8/dist-packages/ < /tmp/rl_coach.patch

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G audio $USERNAME \
    && usermod -a -G video $USERNAME

# Persist history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
    && mkdir /commandhistory \
    && touch /commandhistory/.bash_history \
    && chown -R $USERNAME /commandhistory \
    && echo $SNIPPET >> "/home/$USERNAME/.bashrc"

ENTRYPOINT ["/bin/bash", "source /opt/ros/noetic/setup.bash"]