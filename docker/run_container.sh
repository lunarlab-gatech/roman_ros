DATA_DIR='/media/dbutterfield3/T71'
ROS_WS_DIR='/home/dbutterfield3/Research/ros_workspaces/roman_ros2_ws'

docker run -it \
    --name="roman_ros_hercules" \
    --net="host" \
    --privileged \
    --gpus="all" \
    --workdir="/home/$USER/roman_ros_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --volume="$ROS_WS_DIR:/home/$USER/roman_ros_ws" \
    --volume="$DATA_DIR:/home/$USER/data" \
    --volume="/home/$USER/.bash_aliases:/home/$USER/.bash_aliases" \
    --volume="/home/$USER/.ssh:/home/$USER/.ssh:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTHORITY:/tmp/.Xauthority:ro" \
    roman_ros_hercules \
    bash
