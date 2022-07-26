

echo "CONFIG_IMAGE_KEY=\"spdlog.humble.nav2.flystereo\"" > ./src/isaac_ros_common/scripts/.isaac_ros_common-config
echo "CONFIG_DOCKER_SEARCH_DIRS=($('pwd')/src/oakd_s2/docker/)" >> ./src/isaac_ros_common/scripts/.isaac_ros_common-config


./src/isaac_ros_common/scripts/run_dev.sh `pwd` "-v /dev/bus/usb:/dev/bus/usb"
