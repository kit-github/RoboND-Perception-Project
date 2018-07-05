

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

 - Used statistical filter with mean 30 and noise sigma of 0.25 to remove the outliers. Anything beyond mean_distance + std_dev*noise_thres is considered outlier and removed. The filtered cloud doesn't have the local outlier points in it.![noisy](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/cloud_with_noise.png) and [filtered](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/filtered_point_cloud.png)


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
- **Downsample pointcloud** Downsampled the voxel_grid using leaf size of 0.01 but can be made lower to increase resolution. Mainly done for computational efficiency since original pointcloud can be very dense. See ![Downsampled point cloud](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/downsampled_point_cloud.png)


- **PassThrough Filter** We then use passthrough filter to remove anything that is not interesting to us, like table leg and other stuff. We are mainly interested in the objects on the table top. See ![region of interest](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/roi_point_cloud.png)

 -**Plane Fitting** Since we know we have dominant points coming from table, we use ransac to find a plane. Points that lie within plane are table points and outliers are the objects we care about.See ![table point cloud](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/table_point_cloud.png) and ![object point cloud](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/object_point_cloud.png)


 -**Clustering**  Next we use dbscan to cluster these objects, based on euclidean distance and min and max cluster sizes.  See function segment_objects in project.py. Here are outputs of ![clustering](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/cluster_point_cloud.png)

 -**Individual Objects** The cluster indices from clustering algo is used for getting individual objects. Oublish the point clouds

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

- **Training Object Classifier** 
 - Took 500 samples to capture features.
 - Used linear svm with default parameters
 - Used HSV color space
 - Increased number of bins to 64 from 32 (for both the color and normals)
 - Trained the network using train_svm script
 - Below are the results of training the network
 ![Accuracy - console snapshot](https://github.com/kitu2007/RoboND-Perception- Project/blob/master/data/train/train_performance.png)
 ![confusion matrix - normalized](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/train/normalized_train_performance.png)
 ![confusion matrix - un-normalized] (https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/train/un_normalized_train_performance.png)

 
 - **Test Time** 
  During the test time, loaded the trained SVM model in the main function. There were issues with model giving good results on validation but not working well at test time. Please see section below for more details 'Issues with Classifier Generalization and Fixes'
  - for each cluster
     - compute histogram and normal features
     - concatenate them
     - transform the feature using scaler.transform and call predict
     - publish the label (above the object)
     - create detected object and populate the label and cloud fields


### Pick and Place Setup
    - Now for all the detected objects
    - Read the list of objects to be picked and also
    - Find the location where the object needs to be placed by listening to '/dropbox' topic
    - create a dictionary with group name since green group goes to right bin
    - Populate fields needed for output messages
       - test_scene_num remains the same.
       - object_name, arm_name, pick_pose and place_pose depends on each object and in created in the loop.
       - we create the centriods for each of the object, which is used to assign the pick_pose of the object.
       - the place_pose is determined by the position of the right and left bin centers. The bin in which object is placed depends on the group an object belongs to.
       - create yaml dictionary using the helper function and append them in a list
       - write the list to the yaml_file. Each scene gets its own output file

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

For all the three setup, I ran the above code and dumped the .yaml files and the screen shots. Please see the mentioned .yaml file and the screen shots. 
The output of the three setup is recorded in the yaml files below
![output_1.yaml](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_exercise/output_1.yaml)
![output_2.yaml](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_exercise/output_2.yaml)
![output_3.yaml](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_exercise/output_3.yaml)

 Here are the screen shots with the labels shown in rviz 
![scene1](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_exercise/test_scene_1.png)
![scene2](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_exercise/test_scene_2.png)
![scene3](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_exercise/test_scene_3.png))

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  

**Issues with Classifier Generalization and Fixes** 
Had issues with classifier not working well at test time, even though the classifier performed well on validation set. It was failing to generalize well to all the three scenes. I read some comments suggesting that the test scenario is different from the training scenario. I made several changes to handle the differences. which are mentioned here. 
- First I caputred many more samples to handle pose variation and occlusion during test time. 
- Reduced downsampling by reducing leaf size to 0.005 from 0.01. This made the condition at test time similar to training where object models may have been captured at higher resolution. 
- Also, played with the number of bins used in creating histogram. Increased the number of bins from 32 to 64 which improved results. Also, tried to 128 bins, which seems to also give good results but seemed to be overfitting.
- With increased bin also had to increase the sample size from 50 100 to 300 and finally 500. Also, since the poses can be quite different at test times having more samples is better. More data also helps with recognition in cluttered environment. Best would have been to train with part of model occlued. 
- Tried different kernels, 'rbf', 'sigmoid'. They weren't any better than 'linear', so stuck with linear kernel. 
- Tried different penalty coefficient for SVM. At the end chose C=10. T

**Parameter Tuning for Clustering and Statistical Noise Removal**
- Had few issues with deciding on right parameters for dbscan. How to know the min and max cluster sizes. Played around with the dbscan to find the right parameters. ClusterTolerance to 0.025, min and max cluster sizes to be 50 and 20000.
- Played around for the noise parameters. Even though many settings of num of points for calculating mean and scale factor for std_dev worked, it was hard to say where to draw the line. Experimented with it and settled to 30 points for computing mean and std_dev scale of 0.25. 
      
 

