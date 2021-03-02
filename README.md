
# SDCND : Scan Matching Localization


In this project, we localize a car driving in carla simulation using lidar scan matching.


To run the project, simply run the script `run-carla.sh` to start the carla simulation server, and then, in a new terminal,  run command ./build/cloud_loc.

A recorded scan matching localization video is also provided in the `scan_matching_loc.mp4` file under project directory


## Presentation & Compilation

### Code explaination

Lidar scan matching algorithm codes are located in `if(!new_scan){... }` section of the `c3-main.cpp` file.

### Compilaton
To build the project, run below commands under project directory,
1. mkdir build && cd build
2. cmake ..
3. make -j

The built executable is named cloud_loc.

## localization

As demonstrated in the recorded video `scan_matching_loc.mp4` under project directory, the car is able to drive over 170m, with medium speed (3 taps) and finish the run with a maximum error of 1.07m.

To implement the lidar scan matching with the map, below steps are performed:

1, Voxel filtering  
To speed up ICP optimization, we applied voxel grid filtering to the scan cloud.   

Codes are as below,  

```
pcl::VoxelGrid<PointT> sor;
sor.setInputCloud (scanCloud);
sor.setLeafSize (0.2f, 0.2f, 0.2f);
sor.filter (*cloudFiltered);
```
2, ICP optimization   

Two tricks are applied to achieve better optimization result.    

1) Use motion model to predict vehicle postion so that a more accruate initial guess can be used as the input of ICP optimizaton   
2) When a large residual error is detected after ICP optimization . we assume that current estimation is a bit off ground truth, and thus apply more iterations for next ICP optimization cycle.   

Codes are as below,   

```
pcl::IterativeClosestPoint<PointT, PointT> icp;
icp.setInputSource(cloudFiltered);
icp.setInputTarget(mapCloud);
icp.setMaximumIterations (15);
if(last_icp_score > 0.04){
	//if we detect that last icp iteration optimizaion is a bit off, increase MaximumIterations
	std::cout <<"set larger max itermation"<< std::endl;
	icp.setMaximumIterations (25);
}

pcl::PointCloud<pcl::PointXYZ> Final;
Eigen::Matrix4f guess;
auto pred_pose = pose;
if(last_sim_time !=-1){
	//After the first lidar frame, we will use the simple constant velocity motion model to
	//predict the vehicle position. The motivation behind this step is that icp will have
	//a better initial value and thus achieve better result.
	pred_pose.position.x = pose.position.x + dt * vx;
	pred_pose.position.y = pose.position.y + dt * vy;
}
guess = convert2Eigen(pred_pose);

icp.align(Final, guess);
std::cout <<"icp time="<< tmr.elapsed() << std::endl;
last_icp_score = icp.getFitnessScore();
std::cout << "has converged:" << icp.hasConverged() << " score: " << last_icp_score<< std::endl;
Matrix4f transformation_matrix = icp.getFinalTransformation();
pose = getPose(transformation_matrix.cast <double>());
```

3, scan visualization

This part is quite straightforward, just transform the current scan from vehicle frame to map frame.

```
pcl::transformPointCloud (*scanCloud, *scanCloud, convert2Eigen(pose));
```

## 3D Scan Matching

As requried by project rubric, the vehicle ground truth pose is only used at the beginning of the localization so that ground truth trajectory is relative to initial vehicle pose. Only current lidar scan and map information is used to localize the car.


## Reflection

Using lidar to localize a vehicle is a fun project to work on. There are a few points, in hindsight, worth point out.

1. Setting up Carla on my local computer and make sure that it behaves the same as Udacity workplace takes a bit time, as I am not sure if any custom commands has been run on Udacity workspace to change default Carla server configurations.  
2. I think the Udacity starter code may have a small bug, in that it calcuates pose error before applying scan matching. As a result, we are comparing ground truth pose with last scan matching estimation, instead of current scan matching estimation.  
3. The implicit coordinate frame transformation is a bit confusing. Normally lidar scan matching will help us get the lidar pose relative to map, and an additonal lidar to vehicle transfrom needs to be applied to get vehicle pose, but currently vehicle pose is obtained right after scan matching. As a result, I assume some coordiante frame has already been applied to the map cloud extracted from Carla. As this is not explictly pointed out in project instruction, it takes me qutie a while to figure out.  

4. The 1 meter max pose error is a bit larger than I expected, I am not sure if this has anything to the point 3 mentioned above.  





