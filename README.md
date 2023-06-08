### setup
1. `webots` is the Webots project directory
   - you can open `webots\worlds\ur5e.world` in Webots
2. `ros` has `ros` packages.
    - you can make symbolic links in your local catkin workspace `src` referencing `ros` packages in this repository

### startup procedure
1. `roscore`
2. `webots`
   1. open orange_picking.wbt
   2. start a simulation
3. `roslaunch ur5e_moveit_config move_group.launch load_robot_description:=true`
4. `roslaunch ur_e_webots ur5e.launch`
   1. make sure `export WEBOTS_HOME=/usr/local/webots`
5. `rosrun ur5e_control orange_detection.py`