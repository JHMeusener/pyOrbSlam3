# Image taken from https://github.com/turlucode/ros-docker-gui
FROM osrf/ros:humble-desktop-full-jammy

RUN apt-get update

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y gnupg2 curl lsb-core vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5

RUN apt-get install -y \
    # Base tools
    cmake \
    build-essential \
    git \
    unzip \
    pkg-config \
    python3-dev \
    # OpenCV dependencies
    python3-numpy \
    # Pangolin dependencies
    libgl1-mesa-dev \
    libglew-dev \
    libpython3-dev \
    libeigen3-dev \
    apt-transport-https \
    ca-certificates\
    software-properties-common

RUN apt update


# Build OpenCV
RUN apt-get install -y python3-dev python3-numpy python2-dev
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
RUN apt-get install -y libgtk-3-dev

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && git clone https://github.com/opencv/opencv_contrib &&\
    cd opencv_contrib && git checkout 4.6.0 && cd .. &&\
    cd opencv && git checkout 4.6.0 && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules /tmp/opencv_contrib -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/opencv && rm -rf /tmp/opencv_contrib

# Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.9.1 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++14 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j8 && make install && \
    cd / && rm -rf /tmp/Pangolin

# Build ORB-SLAM3 with its dependencies.
RUN apt-get update && apt-get install ros-humble-pcl-ros tmux -y
RUN apt-get install ros-humble-nav2-common x11-apps nano -y

ARG src1="./pyOrbSlam3"
ARG trg1="/home/pyOrbslam3"
COPY ${src1} ${trg1}

RUN cp -rf /home/pyOrbslam3/modules/changes_in_ORBSLAM3 /home/pyOrbslam3/modules/ORB_SLAM3

RUN apt-get update

RUN echo "Configuring and building Thirdparty/DBoW2 "
RUN . /opt/ros/humble/setup.sh && mkdir /home/pyOrbslam3/modules/ORB_SLAM3/Thirdparty/DBoW2/build && cd /home/pyOrbslam3/modules/ORB_SLAM3/Thirdparty/DBoW2/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8

#RUN echo "Configuring and building Thirdparty/g2o "
#RUN . /opt/ros/humble/setup.sh && mkdir /home/pyOrbslam3/modules/ORB_SLAM3/Thirdparty/g2o/build && cd /home/pyOrbslam3/modules/ORB_SLAM3/Thirdparty/g2o/build && cmake .. -#DCMAKE_BUILD_TYPE=Release && make -j8

RUN echo "Configuring and building Thirdparty/Sophos "
RUN . /opt/ros/humble/setup.sh && mkdir /home/pyOrbslam3/modules/ORB_SLAM3/Thirdparty/Sophus/build && cd /home/pyOrbslam3/modules/ORB_SLAM3/Thirdparty/Sophus/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8

RUN echo "Extracting Vocabulary "
RUN  cd /home/pyOrbslam3/modules/ORB_SLAM3/Vocabulary && tar -xf ORBvoc.txt.tar.gz

RUN echo "Building OrbSlam3"
RUN . /opt/ros/humble/setup.sh && mkdir /home/pyOrbslam3/modules/ORB_SLAM3/build && cd /home/pyOrbslam3/modules/ORB_SLAM3/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8

#install Redis Server
RUN apt-get install lsb-release curl gpg -y
RUN curl -fsSL https://packages.redis.io/gpg | sudo gpg --dearmor -o /usr/share/keyrings/redis-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/redis-archive-keyring.gpg] https://packages.redis.io/deb $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/redis.list
RUN apt-get update && apt-get install redis -y

#install pyOrbSlam
RUN . /opt/ros/humble/setup.sh && pip3 install ninja redis redis-streams && cd /home/pyOrbslam3 && pip3 install .


