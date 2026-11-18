# Project_CALIDAS  -  Camera Lidar Depth Autonomous System

Three simulated autonomous-vehicle projects (ROS 2 Jazzy, Gazebo Harmonic, Linux Mint).

| # | Vehicle | Sensor | Brain (ONE file each) | Launch file |
|---|---|---|---|---|
| 1 | Ackermann car, flat 2D track | camera + OpenCV | `scripts/autonomous_driver.py` | `track_launch.py` |
| 2 | Ackermann car, 3D Silverstone | lidar (+ SLAM, Nav2) | `scripts/autonomous_racer.py` | `silverstone_launch.py` |
| 3 | "auto" (diff drive), city | depth camera (+ RTAB-Map) | `scripts/auto_explorer.py` | `city_launch.py` |

## 0. One-time setup

```bash
sudo apt update
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-rtabmap-ros \
                 ros-jazzy-nav2-smac-planner ros-jazzy-nav2-regulated-pure-pursuit-controller \
                 ros-jazzy-teleop-twist-keyboard python3-matplotlib

cd ~/Documents/Project_CALIDAS
colcon build --packages-select Project_CALIDAS
source install/setup.bash
```
Rebuild (`colcon build ...`) every time you change a file, then `source install/setup.bash`.

## 1. Project 2 - lidar car

```bash
ros2 launch Project_CALIDAS silverstone_launch.py
```
* **Lap 1** the car maps the track slowly with its lidar. SLAM Toolbox builds a 2D map at the
  same time and, when lap 1 ends, it is saved to `~/maps/silverstone.yaml` + `.pgm`.
* **Lap 2+** it races the best line it computed and gets faster every lap until lap times stop improving.
* Lap times: RViz text above the car, `ros2 topic echo /race/lap_time`, and `~/race_data/lap_times_*.csv`
  (graph: `ros2 run Project_CALIDAS plot_laps.py`).

Drive it yourself instead:  `ros2 launch Project_CALIDAS silverstone_launch.py racer:=false`
and in another terminal `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
(press **u** / **o** to steer - a car cannot turn without moving forward).

### Nav2 (click a goal, the car drives there)

```bash
# live map + navigation at the same time
ros2 launch Project_CALIDAS silverstone_launch.py racer:=false nav2:=true
# navigation on the SAVED map
ros2 launch Project_CALIDAS silverstone_launch.py racer:=false slam:=false nav2:=true map:=$HOME/maps/silverstone.yaml
```
In RViz press **2D Goal Pose**, click on free track a few metres ahead and drag to set a direction.
(In the saved-map mode the car must start where it started when the map was made.)

## 2. Project 3 - depth camera "auto"

```bash
ros2 launch Project_CALIDAS city_launch.py
```
It drives by itself (`auto_explorer.py`) while RTAB-Map builds a 3D point-cloud map and a 2D map.
Stop with **one** Ctrl-C and wait: the map is written to `~/maps/city.db`.
Re-use it:  `ros2 launch Project_CALIDAS city_launch.py localization:=true explore:=false`
View the saved 3D map later:  `rtabmap-databaseViewer ~/maps/city.db`

## 3. Why things did not work before (and what was changed)

**Depth camera = black screen**
1. `type="depth"` is not a Gazebo Harmonic sensor, so no sensor existed. It is now `rgbd_camera`.
2. The image format was colour (R8G8B8); depth needs a float image. `rgbd_camera` handles that.
3. The depth image and point cloud were never bridged to ROS. `config/gz_bridge.yaml` now has them.
4. `city.sdf` had no ground: the robot fell forever and the camera saw only sky. A ground plane was added.
5. The DiffDrive plugin used `wheel_radius 0.05` but the wheels are `0.094`, so odometry was wrong by
   almost 2x and no map could line up. Fixed in `gazebo_control_auto.xacro`.

**Nav2 / SLAM not working for the car**
1. SLAM Toolbox is configured with `base_frame: base_footprint`, but the car had no such frame.
   `car.xacro` now has `base_footprint` at the rear axle.
2. Nav2's default settings are for a differential-drive robot (planner NavFn, controller that
   spins on the spot). A car cannot do that. `config/nav2_params.yaml` uses the Smac Hybrid planner
   (minimum turning radius) and Regulated Pure Pursuit with rotate-to-heading OFF, plus the car's footprint.
3. In Jazzy the commands pass through two extra nodes (`velocity_smoother` and `collision_monitor`)
   before `/cmd_vel`; the config keeps them consistent.

## 4. Checklists when something is wrong

**Lidar / SLAM / Nav2**
```bash
ros2 topic hz /scan                          # about 10 Hz
ros2 run tf2_tools view_frames               # make frames.pdf: map -> odom -> base_link -> base_footprint -> laser_frame
ros2 topic echo /map --once | head           # SLAM is producing a map
ros2 lifecycle get /planner_server           # should say  active [3]   (same for /controller_server, /bt_navigator)
ros2 topic echo /cmd_vel                     # Nav2 is sending commands while navigating
```
* `Timed out waiting for transform ... map`  -> SLAM Toolbox (or AMCL) is not running / no map yet.
* `Failed to create plan` -> the goal is on an obstacle or unknown space, or closer than the turning radius allows. Choose a goal 3-5 m away on open track.
* The car sits still but Nav2 says it is moving -> check `ros2 topic echo /cmd_vel` and that the Gazebo window is not paused.
* The 3D Silverstone mesh has slopes: a flat 2D lidar sees a steep ramp as a wall. Navigation works best on the flat parts.

**Depth camera**
```bash
gz topic -l | grep depth_camera              # 4 topics: image, depth_image, points, camera_info
gz topic -e -t /depth_camera/depth_image -n 1   # prints numbers, not an error
ros2 topic hz /depth_camera/depth_image      # about 10 Hz
ros2 run rqt_image_view rqt_image_view       # pick /depth_camera/depth_image
```
* No `gz topic` lines -> the sensor was not created: run `xacro description/full_auto.urdf.xacro | grep rgbd_camera`.
* Topics exist in Gazebo but not in ROS -> the bridge config was not rebuilt (`colcon build`).
* Image is black but data is flowing -> the robot is facing a wall or is inside a building. Change `SPAWN_X/Y/YAW` in `city_launch.py`.
