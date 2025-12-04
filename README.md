
# Running program
## Setting up algorithm
- cd CSCE-452-Project-4/
- colcon build
- source install/setup.bash
- ros2 launch tile_localization p4.launch.py world_file:=windy.world bag_file:=20-11

## Rviz2 Setup
- * In a new terminal
- Set Fixed Frame to 'map'
- Add Map
- Set Topic = '/floor' and Update Topic = '/floor_updates'
- Add Marker
- Set Topic = '/position_marker'

## Running bag files
- * In a new terminal
- cd CSCE-452-Project-4/
- ros2 bag play 13/ 
- * replace the number with the corresponding bag for the world