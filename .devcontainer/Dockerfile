FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND="noninteractive" 
ARG USERNAME=user-name-goes-here
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt update && \ 
    apt upgrade -y && \
    apt install --no-install-recommends -y \
    apt-utils \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    dirmngr \
    ffmpeg \
    git \
    gnupg2 \
    jq \
    jwm \
    libcurl4-openssl-dev \
    libavcodec-dev \
    libavformat-dev \
    libboost-all-dev \
    libfreetype-dev \
    libgmock-dev \
    libgomp1 \
    libhdf5-dev \
    libjpeg-dev \
    liblog4cplus-dev \
    libpng-dev \
    libssl-dev \
    libswscale-dev \
    libx264-dev \
    libxrender1 \
    libzmq3-dev \
    lsb-release \
    mesa-utils \
    net-tools \
    pkg-config \
    python3-dev \
    python3-opengl \
    python3-pip \
    tzdata \
    uuid-dev \
    unzip \
    wget \
    x11-xserver-utils \
    x11vnc \
    x264 \
    xmlstarlet \
    xvfb \
    zlib1g-dev \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-defusedxml \
    python3-jsonschema \
    python3-rosdep \
    python3-shapely \
    python3-vcstool \
    python3-boto3 \
    ros-jazzy-controller-manager \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-base \
    ros-jazzy-ros-core \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-controllers \
    ros-jazzy-web-video-server \
    ros-jazzy-xacro && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Build and install AWS SDK C++
RUN git clone --recurse-submodules https://github.com/aws/aws-sdk-cpp.git && \
        cd aws-sdk-cpp && git checkout --recurse-submodules tags/1.11.664 && \
        mkdir build && cd build && \
        cmake .. \
            -DCMAKE_BUILD_TYPE=Release \
            -DCPP_STANDARD=17 \
            -DCMAKE_PREFIX_PATH=/usr/local \
            -DCMAKE_INSTALL_PREFIX=/usr/local \
            -DBUILD_SHARED_LIBS=ON \
            -DENABLE_TESTING=OFF \
            -DBUILD_ONLY="core;logs;kinesis" \
            -DMINIMIZE_SIZE=ON && \
        make -j2 && \
        make install && \
        ldconfig && \
        cd ../.. && rm -rf aws-sdk-cpp

# Install AWS CLI v2
RUN curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip" && \
    unzip awscliv2.zip && \
    ./aws/install && \
    rm -rf aws awscliv2.zip
        
# Create the user
RUN userdel ubuntu && groupadd --gid $USER_GID $USERNAME \
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

RUN mkdir -p /opt/amazon/configs
COPY .devcontainer/environment.yml /opt/amazon/configs/environment.devcontainer.yml

USER $USERNAME
ENV CONDA_DIR=/home/$USERNAME/miniconda

# Install Miniconda as non-root
RUN curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-py312_24.7.1-0-Linux-$(uname -m).sh -o /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p "$CONDA_DIR" && \
    rm /tmp/miniconda.sh 

# Create sagemaker_env conda environment from environment.devcontainer.yml
RUN ${CONDA_DIR}/bin/conda env create -f /opt/amazon/configs/environment.devcontainer.yml -n sagemaker_env --yes || \
    ${CONDA_DIR}/bin/conda env update -f /opt/amazon/configs/environment.devcontainer.yml -n sagemaker_env && \
    ${CONDA_DIR}/bin/conda clean -afy && \
    ${CONDA_DIR}/bin/conda config --set auto_activate_base false

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec /bin/bash"]