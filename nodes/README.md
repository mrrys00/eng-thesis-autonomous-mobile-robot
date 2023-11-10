# TEMPORARY

copy nodes to destination machine - RPi or VM - via scp (described in the simulator readme)

clear all cache: `rm -rf build/ log/ install/`
build only exploration node: `colcon build --packages-select exploration_algorithm`
run exploration: `ros2 run  exploration_algorithm exploration_algorithm_node`

It's necessary to improve not explored points detection!!!
That doesn't work!
