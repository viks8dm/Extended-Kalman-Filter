# Extended Kalman Filter

In this project-1 of term-2 of self driving car nanodegree program by Udacity a Kalman Filter is utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements.
For satisfactory project completion, the requirement is that the RMSE values obtained should be lower than the target-RMSE values outlined in project ruberic, details of which are available on [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)

## Contents of this repository

The project has been created using Udacity's [starter Code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)




* `src/main.cpp` - communicates with the [simulator](https://github.com/udacity/self-driving-car-sim/releases/); receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `src/FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
* `src/kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar
* `src/tools.cpp` - function to calculate RMSE and the Jacobian matrix
* `data` - folder with input file with data corresponding to dataset-1 of the simulator (I plan to implement a file I/O architechture, so that simulator is not needed in future)
* `results` - directory that contains any results (or output files, once file I/O architechture is ready)
* `Docs` - directory containing documents explaining I/O file format and explaning data flow structure within the source-code.

## Results & Discussion

[image1]: ./results/Dataset1_FullPath.jpeg "full path for dataset-1 with RSME values"
[image2]: ./results/Dataset2_FullPath.jpeg "full path for dataset-2 with RSME values"

The RMSE-target criteria for dataset-1 was given as: RMSE <= [.11, .11, 0.52, 0.52] 

As shown in the screenshots below. Both for dataset-1 and dataset-2, the final RMSE values satisfy this criteria:

* Dataset-1: RMSE = [0.10, 0.09, 0.45, 0.44]
* Dataset-1: RMSE = [0.07, 0.10, 0.42, 0.49]


--

![alt text][image1]
--

![alt text][image2]


## To run the code

Clone this repository and enter following commands in a terminal

`mkdir build && cd build`

`cmake .. && make`

`./ExtendedKF`

After execution of `./ExtendedKF`, simulator should be opened and it should be started with dataset of interest selected, as shown in the screenshots above. 




