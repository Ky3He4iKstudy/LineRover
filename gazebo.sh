docker run --gpus all -ti --rm -e "DISPLAY" \
               -e "QT_X11_NO_MITSHM=1" \
--env="DISPLAY=$DISPLAY"  \
--env="QT_X11_NO_MITSHM=1" \
--volume="$XAUTHORITY:$XAUTHORITY" \
--env="XAUTHORITY=$XAUTHORITY" \
--runtime=nvidia \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
               -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
               -e XAUTHORITY \
               -v $(pwd)/catkin_ws:/catkin_ws \
               --net=host \
               --privileged \
               --name itmo-ros-cont itmo-ros-aaa

