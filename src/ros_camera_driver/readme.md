# Commands

Para executar o Docker:

```bash
docker run -it --rm \
    --name camera_node \
    --net=host \
    --privileged \
    ros_camera_pi:jazzy
```

Para executar o nó que publica imagens: 

```bash
export ROS_DOMAIN_ID=42

ros2 run camera_ros camera_node --ros-args -p width:=960 -p height:=720 -p camera_info_url:="file:///camera_ws/camera.yaml" -p Sharpness:=16.0 -p ColourTemperature:=100000 -p Saturation:=0.0
```

