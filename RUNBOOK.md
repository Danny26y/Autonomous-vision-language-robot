# Rover: daily start (one terminal per line, in order)
T1  ros2 launch rover_sim rover_world.launch.py gui:=false
    ready when you see: Successfully spawned entity [waffle]
T2  ros2 launch nav2_bringup localization_launch.py use_sim_time:=true map:=$HOME/turtlebot3_map.yaml
    ready when you see: Managed nodes are active, then in a new terminal: ~/rover_ws/set_pose.sh
T3  ros2 run rover_perception perception_node --ros-args -p use_sim_time:=true -p detector:=yolo -p "classes:=fire hydrant" -p imgsz:=320
    ready when you see lines starting: fire hydrant 0.xx
T4  ros2 run rover_localize target_locator --ros-args -p use_sim_time:=true
    working when you see: fire hydrant @ map = (x, y, z)
Checks: ros2 run tf2_ros tf2_echo map odom   |   ros2 topic echo /perception/target_map --once
Stop: Ctrl+C in each terminal, then: pkill -f gzserver
Add gui:=true to T1 to see Gazebo (slower).
Note: after T3/T4 start, allow up to 2 minutes for the first detection on this laptop.
Full Nav2 bringup: set the initial pose as soon as AMCL warns, then wait for the second 'Managed nodes are active'.
