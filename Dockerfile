#ARG from
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils \
                                          lsb-release \
                                          mesa-utils \
                                          gnupg2 \
                                          net-tools \
                                          build-essential \
                                          wget \
                                          unzip \
                                          curl \
                                          git \
                                          mc wget

# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS desktop and Gazebo
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    curl -s https://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full gazebo11 \
        libgazebo11-dev ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
        ros-noetic-controller-manager ros-noetic-gazebo-ros \
        ros-noetic-gazebo-plugins ros-noetic-gazebo-msgs  \
        python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator \
        python3-wstool ros-noetic-catkin \
        && \
    mkdir -p /catkin_ws/src && \
    echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash"  >> ~/.bashrc
#    cd /catkin_ws/src && catkin_init_workspace && cd .. && catkin_make && \
#    rosdep init && rosdep update && 
#    rosdep install --from-paths . --ignore-src --rosdistro noetic -y && \





#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
#    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
#    apt-get update && DEBIAN_FRONTEND=noninteractive \
#                      apt-get install -y ros-melodic-desktop-full \
#                                         gazebo9 \
#                                         ros-melodic-gazebo-ros-pkgs \
#                                         ros-melodic-ros-control \
#                                         ros-melodic-gazebo-ros-control \
#                                         ros-melodic-geographic-info \
#                                         ros-melodic-teleop-twist-keyboard \
#                                         ros-melodic-joy \
#                                         ros-melodic-effort-controllers \
#                                         ros-melodic-controller-manager \
#                                         python-rosdep \
#                                         python-rosinstall \
#                                         python-rosinstall-generator \
#                                         python-wstool \
#                                         python-catkin-tools \
#                                         libcanberra-gtk-module \
#                                         libcanberra-gtk3-module \
#                                         ros-melodic-pid \
#                                         ros-melodic-visp* && \
#    rosdep init && rosdep update && \
#    echo "source /opt/ros/melodic/setup.bash"  >> ~/.bashrc && \
#    echo "source /catkin_ws/devel/setup.bash"  >> ~/.bashrc

# Install extra libraries
#RUN apt-get install -y libvisp-dev libvisp-doc 
RUN apt-get install -y python3-pip
RUN pip install pathlib statistics scipy

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-noetic-ecl-geometry \
                                            ros-noetic-tf2-sensor-msgs \
                                            ros-noetic-move-base-msgs \
                                            ros-noetic-image-proc

RUN apt-get update && apt-get install -y \
                libopencv-dev libx11-dev liblapack-dev libv4l-dev libzbar-dev libpthread-stubs0-dev libsdl-dev libsdl-image1.2-dev \
                libeigen3-dev

RUN pip install numpy scikit-image 
RUN pip install pillow 
RUN pip install opencv-contrib-python

RUN apt-get install nano
RUN ln -s /bin/python3 /bin/python
RUN echo "cd /catkin_ws" >> /root/.bashrc
