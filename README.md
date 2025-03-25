# ros2 Coursework

## Running instructions
- make sure you are in a ros2 singularity environment
- run `colcon build` to build the function
- in a seperate terminal window run `ros2 launch turtlebot3_gazebo turtlebot3_task_world_2025.launch.py` to launch the simulator
- in a seperate terminal window run `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_sc21hkw/map/map.yaml` to launch the map
- estimate the starting pose using the '2D Pose Estimate tool' in the map
- return to the original terminal window and run `ros2 run ros2_project_sc21hkw explore_map` to start the program