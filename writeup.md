

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

 - Used statistical filter with mean 30 and noise sigma of 0.25 to remove the outliers. Anything beyond mean_distance + std_dev*noise_thres is considered outlier and removed. The filtered cloud doesn't have the local outlier points in it.
 Noisy ![noisy](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/cloud_with_noise.png) and Filtered ![filtered](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/filtered_point_cloud.png)


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
- **Downsample pointcloud** Downsampled the voxel_grid using leaf size of 0.01 but can be made lower to increase resolution. Mainly done for computational efficiency since original pointcloud can be very dense. See Downsampled point cloud ![Downsampled point cloud](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/downsampled_point_cloud.png)


- **PassThrough Filter** We then use passthrough filter to remove anything that is not interesting to us, like table leg and other stuff. We are mainly interested in the objects on the table top. Here only portion of table of interest is shown ![region of interest](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/roi_point_cloud.png)

 -**Plane Fitting** Since we know we have dominant points coming from table, we use ransac to find a plane. Points that lie within plane are table points and outliers are the objects we care about. See table top point cloud ![table point cloud](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/table_point_cloud.png) and object point cloud ![object point cloud](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/object_point_cloud.png)


 -**Clustering**  Next we use dbscan to cluster these objects, based on euclidean distance and min and max cluster sizes.  See function segment_objects in project.py. Here are outputs of clustering ![clustering](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/exercises/cluster_point_cloud.png)

 -**Individual Objects** The cluster indices from clustering algo is used for getting individual objects. Oublish the point clouds

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

- **Training Object Classifier** 
The classifier was trained with many different combination of parameters. Although most of them setting performed well on validation, I had to made extra adjustment to make sure the model performs well at test time. The final parameters here are coming from some of these choices I made for imporving generalization of classifier. Please see section 'Issues with Classifier Generalization and Fixes' below for more details. 
 - Took 500 samples to capture features.
 - Used linear svm with default parameters
 - Used HSV color space
 - Increased number of bins to 64 from 32 (for both the color and normals). This increased the performance. Increasing to 128 worked well but was overfitting. 
 - Trained the network using train_svm script
 - increased the SVM penalty C to 10. 

- Below are the results of training the network
 ![Accuracy - console snapshot](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/train/train_performance.png)
 ![confusion matrix - normalized](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/train/normalized_train_performance.png)
 ![confusion matrix - un-normalized](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/train/un_normalized_train_performance.png)


 
 - **Test Time** 
  During the test time, loaded the trained SVM model in the main function. There were issues with model giving good results on validation but not working well at test time. Please see section below for more details 'Issues with Classifier Generalization and Fixes'
  - for each cluster
     - compute histogram and normal features
     - concatenate them
     - transform the feature using scaler.transform and call predict
     - publish the label (above the object)
     - create detected object and populate the label and cloud fields


### Pick and Place Setup
For the pick and place operation, we need the following steps. 
- first get the list of objects that are supposed to be in a given scene by getting the params by listening to '/object_list'. 
- Then we also listen to '/dropbox' topic to know the location of the red and green bins where the object needs to be placed. We add a little jitter so objects aren't placed on top of each other within each bins. 
- Using this info I create a dictionary with group name as the key. Since the group decides the bin the object is supposed to be placed it serves as a good key.

- With this info we now take the list of detected objects, run the classifier on each of the detected object and if the name of the detected object matches the list of pick objects in the scene, we populate the fields needed for output the .yaml messages. For each of identified object, we create       
- object_name. The label provided by the classifier
- arm_name. Based on the group of the object. 
- pick_pose. Centroid computed by using the mean of the 3d points. 
- place_pose. Location of the left or right bin depending on the group of the object. 

We then create a yaml dictionary using the helper function and append them in a list, which is then written to a yaml_file using the helper function. Please see below for the yaml files for each scene and the screen shots. 

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

For all the three setup, I ran the above code and dumped the .yaml files and the screen shots. Please see the mentioned .yaml file and the screen shots. 
The output of the three setup is recorded in the yaml files below
![output_1.yaml](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_output/output_1.yaml)
![output_2.yaml](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_output/output_2.yaml)
![output_3.yaml](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_output/output_3.yaml)

 Here are the screen shots with the labels shown in rviz 
- Scene1 ![scene1](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_output/test_scene_1.png)
- Scene2 ![scene2](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_output/test_scene_2.png)
- Scene3 ![scene3](https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/final_output/test_scene_3.png))

## Discussion 
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
      
 

