#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)



def create_cluster_mask(white_cloud, cluster_indices):
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            point_item = white_cloud[indice]
            color_cluster_point_list.append([ point_item[0],
                                              point_item[1],
                                              point_item[2],
                                              rgb_to_float(cluster_color[j])
                                            ]
                                           )
    return color_cluster_point_list

def segment_objects(cloud_objects):
    """
    Cluster extraction and create cluster mask
    """
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster sizes (in points)
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(2000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    return cluster_indices, white_cloud


def statistical_filter(pcl_data):

    # TODO: Stastical Outlier Filtering
    # pcl_data = XYZRGB_to_XYZ(pcl_data)
    outlier_filter = pcl_data.make_statistical_outlier_filter()

    # Set the neighboring points to be 50
    outlier_filter.set_mean_k(30)

    # Set the threshold scale factor
    noise_sigma_scale = .25

    # Any point with a distance larger than the global distance
    # mean_distance + std_dev*noise_thres will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(noise_sigma_scale)

    cloud_filtered = outlier_filter.filter()

    return cloud_filtered

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    cloud = statistical_filter(cloud)
    # pcl_cluster_pub.publish(pcl_to_ros(cloud))

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()


    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6; axis_max=1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    if 1:
        passthrough_y = cloud_filtered.make_passthrough_filter()
        filter_axis = 'y'
        passthrough_y.set_filter_field_name(filter_axis)
        axis_min = -.5; axis_max=.5
        passthrough_y.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough_y.filter()

    #ros_cloud_passthrough = pcl_to_ros(cloud_filtered)
    #pcl_table_pub.publish(ros_cloud_passthrough)

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # TODO: Extract inliers and outliers
    max_distance = 0.03
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    if 0:
        pcl.save(cloud_objects, 'cloud_objects.pcd')

    # TODO: Euclidean Clustering
    cluster_indices, white_cloud = segment_objects(cloud_objects)

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    color_cluster_point_list = create_cluster_mask(white_cloud, cluster_indices)
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # TODO: Publish ROS messages


# Exercise-3 TODOs:
    if 1:
        # Classify the clusters! (loop through each detected cluster one at a time)
        detected_objects_labels=[]
        detected_objects = []

        # Grab the points for the cluster
        for index, pts_list in enumerate(cluster_indices):
            # Grab the points for the cluster from the extracted outliers (cloud objects)
            pcl_cluster = cloud_objects.extract(pts_list)
            # TODO: convert the cluster from pcl to ROS
            ros_cluster = pcl_to_ros(pcl_cluster)

            # Compute the associated feature vector
            chists = compute_color_histograms(ros_cluster, using_hsv=True, nbins=64)
            normals = get_normals(ros_cluster)
            nhists = compute_normal_histograms(normals, nbins=64)
            feature = np.concatenate((chists, nhists))

            # Make the prediction
            prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
            label = encoder.inverse_transform(prediction)[0]
            detected_objects_labels.append(label)

            # Publish a label into RViz
            label_pos = list(white_cloud[pts_list[0]])
            #label_pos += .4
            object_markers_pub.publish(make_label(label, label_pos, index))

            # Add the detected object to the list of detected objects.
            do = DetectedObject()
            do.label = label
            do.cloud = ros_cluster
            detected_objects.append(do)

            # Publish the list of detected objects
            rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
            # rospy.loginfo('Detected {}'.format(len(detected_objects_labels))

            detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()

    if 1:
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    labels=[]
    centroids=[]

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    # TODO: Parse parameters into individual variables
    object_name = object_list_param[i]['name']
    object_group = object_list_param[i]['group']

    if 0:
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    # TODO: Loop through the pick list
    for pick_object in object_list_param:
        for object in object_list:
            if object.label == pick_object:
                labels.append(object.label)
                # TODO: Get the PointCloud for a given object and obtain it's centroid
                point_arr = ros_to_pcl(object.cloud).to_array()
                tmp =  np.mean(points_arr, axis=0)[:3]
                tmp[0] = np.asscalar(tmp[0])
                tmp[1] = np.asscalar(tmp[1])
                tmp[2] = np.asscalar(tmp[2])
                centroids.append(tmp)

    # TODO: Create 'place_pose' for the object
    # Parse the dropbox.
    drop_items = rospy.get_param('/dropbox')
    group = drop_items[i]['group']
    position = drop_items[i]['position']

    # TODO: Assign the arm to be used for pick_place
    if group=='green' arm_name == 'right'
    
    # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

    # Wait for 'pick_place_routine' service to come up
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        # TODO: Insert your message variables to be sent as a service request
        resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file



if __name__ == '__main__':

  if 0:
      indir = '/home/robond/data'
      pcd_data = pcl.load('{}/table_scene_lms400.pcd'.format(indir))
      filtered_pcd = statistical_filter(pcd)
      pcl.save(filtered_pcd, "{}/filtered_table.pcd".format(indir))
  if 1:
    # TODO: ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points",
                               pc2.PointCloud2, pcl_callback, queue_size =1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # TODO: additional publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('/home/robond/catkin_ws/model.sav', 'rb'))
    print ("Loaded the model")

    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
