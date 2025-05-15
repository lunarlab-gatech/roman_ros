DATA_DIR='/media/dbutterfield3/T71'

docker start roman_ros_hercules && docker exec -it roman_ros_hercules /bin/bash \
    --volume="$DATA_DIR:/home/$USER/data" \