---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

 - Used statistical filter with mean 30 and noise sigma of 0.25 to remove the outliers. Anything beyond mean_distance + std_dev*noise_thres is considered outlier and removed. The filtered cloud doesn't have the local outlier points in it.


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.
     - Downsampled the voxel_grid using leaf size of 0.01 but can be made lower to increase resolution. This is mainly done for computational efficiency since original pointcloud can be very dense.
     - We then use passthrough filter to remove anything that is not interesting to us, like table leg and other stuff. We are mainly interested in the objects on the table top
     - Since we know we have dominant points coming from table, we use ransac to find a plane. Points that lie within plane are table points and outliers are the objects we care about.
     - Next we use dbscan to cluster these objects, based on euclidean distance and min and max cluster sizes.  See function segment_objects in project.py
     - The cluster indices from clustering algo is used for getting individual objects.
     - publish the point clouds

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
     - Took 300 samples to capture features.
     - Used linear svm with default parameters
     - Used HSV color space
     - Increased number of bins to 64 from 32 (for both the color and normals)
     - Trained the network using train_svm script
     - During the test time, loaded the trained SVM model in the main function
     - for each cluster
        - compute histogram and normal features
        - concatenate them
        - transform the feature using scaler.transform and call predict
        - publish the label (above the object)
        - create detected object and populate the label and cloud fields


Here is an example of how to include an image in your writeup.
     ![exercise-3 - confusion matrix - normalized] (https://github.com/kitu2007/RoboND-Perception-Project/blob/master/data/figure_1.png)
![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

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

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  

      -- The voxel size of 0.01 was too coarse. Increased it to 0.005
      -- Had to play around with the dbscan to find the right parameters. ClusterTolerance to 0.025, min and max cluster sizes to be 50 and 20000.
      -- Same for the noise. Even though many settings of num of points for calculating mean and scale factor for std_dev worked, it was hard to say where to draw the line. Experimented with it and settled to 30 points for computing mean and std_dev scale of 0.25. 
      -- The SVM classifier seemed to perform well in the validation set and worked well for scene1 and scene2 but wasn't as stable for scene3. This got a little frustating. I wasn't sure why it is failing. I guess the test scenario is so different from the training scenario. We don't know what is the density of the point cloud while training. I realized may be I am subsampling too much at test time. So decreased the leaf_size in voxel_filter to 0.005 
      
      - Also, played with the number of bins used in creating histogram
      Increased the number of bins from 32 to 64 which improved results. Also, tried to 128 bins, which seems to also give good results but seemed to be overfitting.
      - With increased bin also had to increase the sample size from 50 to 100 to 300 and finally 500. Also, since the poses can be quite different at test times having more samples is better.
      -- Clutter causes occlusion. During test time we see lot more occlusion but it is not present during capturing features. This makes the model somewhat sensitive to occlusion, especially for small objects.
      -- Tried different kernels, 'rbf', 'sigmoid'. They weren't any better than 'linear', so stuck with linear kernel. 
      -- It was hard to come out with penalty coefficient for SVM, so kept it as default. 

