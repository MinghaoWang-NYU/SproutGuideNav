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
