192.168.200.1
port: 9080
passwords: fauna

press up & XX to stand
press AA to walk


Use docker to publish command:
newgrp docker
docker start -ai ros2_sprout
docker exec -it ros2_sprout bash

ros2 run rmw_zenoh_cpp rmw_zenohd

ros2 topic pub /motor_control/velocity/command     fauna_msgs/msg/VelocityCommand     "{command_priority: {sender_id: 'test', priority: 0}, \
      vx: 0.1, vy: 0.0, vyaw: 0.0}"     --rate 5

ros2 topic echo /motor_control/velocity/command

python3 sensor/extract_data_two.py --output-dir ./data/teaching_run

# DINOv3 adaptive keyframe selection (used in the paper)
python3 topogen/gen_dinov3.py \
    --input ./data/teaching_run/<timestamp>/color \
    --output ./data/topomap_raw \
    --dinov3-repo /path/to/dinov3 --weights /path/to/dinov3_vitl16.pth

# gen_dinov3.py writes keyframe_000000.jpg; rename to the numeric scheme
mkdir -p ./data/topomap
i=0; for f in $(ls -v ./data/topomap_raw/keyframe_*.jpg); do \
    cp "$f" "./data/topomap/$i.jpg"; i=$((i+1)); done

# pre-compute place recognition descriptors (optional; done automatically on first run)
python3 -m guidenav.place_recognition.extract_database --topomap-dir ./data/topomap

# Evaluate offline
source /opt/ros/humble/setup.bash

python3 guidenav/navigate.py \
    --robot mc \
    --robot-config-path ./config/robots.yaml \
    --topomap-base-dir ./data -d topomap \
    --model-weight-dir model_weights \
    --model-config-path config/models.yaml \
    --offline-images --img-dir ./data/teaching_run/20260730_225825/color

# robot deployment
source /opt/ros/humble/setup.bash

python3 guidenav/navigate.py \
    --robot sprout \
    --robot-config-path ./config/robots.yaml \
    --topomap-base-dir ./data -d topomap \
    --model-weight-dir model_weights \
    --model-config-path config/models.yaml