# midterm_localization
This repo is created a ROS package for midterm localization competition of Self-Driving Car course

<!-- |figures || -->

|<img src="https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_1.png" width="300" /><br /><p align="center">itri_map_1</p>|<img src="https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_2.png" width="300" /><br /><p align="center">itri_map_2</p> |
|-----|:-----:|
|<img src="https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_3.png" width="300" /><br /><p align="center"><b>itri_map_3</b></p>|<img src="https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_4.png" width="300" /><br /><p align="center"><b>itri_map_4</b></p> |

# Maps setup
Please put the \*.pcd files to test_data/[MAP_NAME] folder like this:
```bash
├── midterm_localization
└── test_data
    ├─── itri_map
    │    └─── map.pcd
    │
    ├─── nctu_map
    │    ├─── first-0.pcd
    │    └─── first-10.pcd
    │           ...
    └─── nuscene_map
         ├───map_200_600.pcd
         └───map_200_700.pcd
                ...
```

# How to run the code
Note: First run the launch file, then wait the terminal shows the green words **" solution1_node is ready. "**, so that you can run the rosbag.

After compiling(catkin_make), run the code by:  
### ITRI
```bash
roslaunch midterm_localization solution_easy.launch use_rviz:=true \
    map_name:=itri \
    submap_size:=100 \
    local_sizex:=100 local_sizey:=24 \
    num_threads:=8 \
    num_pretest_frames:=4 \
    num_pretest_poses:=200
--------------------------------------------------------
rosbag play ITRI_Private_1.bag --clock -r 0.05
```
### Nuscenes
```bash
roslaunch midterm_localization solution_medium.launch use_rviz:=true \
    map_name:=nuscene \
    submap_size:=100 \
    local_sizex:=100 local_sizey:=100 \
    num_threads:=8 \
    num_pretest_frames:=4 \
    num_pretest_poses:=200 \
    remove_sizey_l:=-20 \
    remove_sizey_h:=10
--------------------------------------------------------
rosbag play Nu_Private_1.bag --clock -r 0.05
```

# The result files
The result is saved in **midterm_localization/csv_files** directory, and the file is named by the map name and the current date time (e.g. itri_20200505T213332.csv)


[itri_map_1]: https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_1.png "itri_map_1"
[itri_map_2]: https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_2.png "itri_map_2"
[itri_map_3]: https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_3.png "itri_map_3"
[itri_map_4]: https://github.com/coolcat647/midterm_localization/blob/master/figures/itri_map_4.png "itri_map_4"
