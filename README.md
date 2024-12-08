# UR5e_CSV_Controller

Create a ROS2 Worspace if one doesn't exist:
```
mkdir ~/ros2ws/src
```

Clone Repo:

```
cd ~/ros2ws/src
```

```
git clone https://github.com/Ranasinghe843/UR5e_CSV_Controller.git
```

Create folder for CSV files in the package and place sample files from [this google drive folder](https://drive.google.com/drive/folders/1MROOuXklGNv8oz4bb8jZSW4jPNivwkmw?usp=sharing):
```
mkdir ~/ros2ws/src/UR5e_CSV_Controller/ur5e_csv_controller/csv_files
```

Build and source workspace:
```
cd ~/ros2ws
```

```
colcon build
```

```
source ~/ros2ws/install/local_setup.bash
```

Run the controller:

```
ros2 launch ur5e_csv_controller ur5e_csv.launch.py launch_rviz:=false csv_name:=ur5e_lissajous
```
