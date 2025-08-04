Running service
ros2 run service_pos get_positions_server
ros2 service call /get_positions service_pos/srv/GetPositions "{}"

Running Node
ros2 run aktual vision
ros2 run aktual communication
ros2 run aktual strategy

Running RVIZ via launch
ros2 launch aktual tcp_bridge_launch.py 
