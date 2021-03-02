
# SDCND : Scan Matching Localization


In this project, we localize a car driving in carla simulation using lidar scan matching.


To run the project, simply run the script `run-carla.sh` to start the carla simulation server, and then, in a new terminal,  run command ./build/cloud_loc.




## Presentation & Compilation

### Code explaination

Lidar scan matching algorithm codes are located in `if(!new_scan){... }` section of the c3-main.cpp file.

### Compilaton
To build the project, run below commands under project directory,
1. mkdir build && cd build
2. cmake ..
3. make -j

The built executable is named cloud_loc.

## localization

As demonstrated in the recorded video `scan_matching_loc.mp4` under project directory, the car is able to drive over 170m, with medium speed (3 taps) and finish the run with a maximum error of 1.07m.

To implement the lidar scan matching with the map, below steps are performed:

1. Voxel filtering
To speed up ICP optimization, we applied voxel grid filtering the the scan cloud.
2. ICP optimization
3. scan visualization

## 3D Scan Matching
We use ekf to track objects. EKF is implemented in `filter.py`.   

Implementing an ekf includes below work, 

* Design system sate [x,y,z,vx,vy,vz] 
* Design process model, consant velocity model
* implement predict step, with constant velociy model and process noise increasing with delta time
* implement upate step,  with lidar measurement model

For the road segment specified in classroom, the rmse score is 0.28.   

<img src="img/ekf_rmse.png"/>




## Reflection

In this section, we address a few topics raised in the classroom.





