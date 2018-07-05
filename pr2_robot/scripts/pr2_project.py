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

nbins=64

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


def segment_objects(white_cloud):
    """
    Cluster extraction and create cluster mask
    """
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster sizes (in points)
    ec.set_ClusterTolerance(0.025)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(20000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    return cluster_indices


def statistical_filter(pcl_data):
    "Filter the pcl data"
    # pcl_data = XYZRGB_to_XYZ(pcl_data)
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    # Set the neighboring points to be 50
    outlier_filter.set_mean_k(20)
    # Set the threshold scale factor
    noise_sigma_scale = 0.1
    # Any point with a distance larger than the global distance
    # mean_distance + std_dev*noise_thres will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(noise_sigma_scale)
    cloud_filtered = outlier_filter.filter()
    return cloud_filtered


def get_roi(pcd_cloud):
    "Get the area around the table"
    passthrough = pcd_cloud.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6; axis_max=1
    passthrough.set_filter_limits(axis_min, axis_max)
    pcd_cloud = passthrough.filter()

    passthrough_y = pcd_cloud.make_passthrough_filter()
    filter_axis = 'y'
    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -.4; axis_max=.4
    passthrough_y.set_filter_limits(axis_min, axis_max)
    pcd_cloud = passthrough_y.filter()
    return pcd_cloud


def downsample_cloud(pcd_cloud):
    vox = pcd_cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    return vox.filter()


def plane_fitting(pcd_cloud):
    seg = pcd_cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # TODO: Extract inliers and outliers
    max_distance = 0.03
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    cloud_table = pcd_cloud.extract(inliers, negative=False)
    cloud_objects = pcd_cloud.extract(inliers, negative=True)
    return cloud_table, cloud_objects

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    filtered_cloud = statistical_filter(cloud)

    # TODO: Voxel Grid Downsampling
    downsampled_cloud = downsample_cloud(filtered_cloud)

    # TODO: PassThrough Filter
    region_cloud= get_roi(downsampled_cloud)

    # TODO: RANSAC Plane Segmentation
    cloud_table, cloud_objects = plane_fitting(region_cloud)


    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)

    cluster_indices = segment_objects(white_cloud)
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    color_cluster_point_list = create_cluster_mask(white_cloud, cluster_indices)
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))
    pcl_table_pub.publish(pcl_to_ros(cloud_table))
    pcl_cluster_pub.publish(pcl_to_ros(cluster_cloud))


    # Exercise-3 TODOs:
    detected_objects_labels=[]
    detected_objects = []

    # Grab the points for the cluster
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # TODO: convert the cluster from pcl to ROS
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True, nbins=nbins)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals, nbins=nbins)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] = label_pos[2] + 0.2
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels),
                                                   detected_objects_labels))
    rospy.loginfo('Detected {}'.format(len(detected_objects_labels)))
    detected_objects_pub.publish(detected_objects)

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
    drop_items = rospy.get_param('/dropbox')

    # create a dictionary with color green as the key
    drop_position = dict()
    for drop_item in drop_items:
        drop_position[drop_item['group']] = (drop_item['name'],drop_item['position'])

    test_scene_num = Int32()
    test_scene_num.data = 3

    dict_list = []

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    # TODO: Loop through the pick list
    for pick_object in object_list_param:
        for object in object_list:
            if object.label == pick_object['name']:
                # if their is a match create msg
                object_name = String()
                arm_name = String()
                pick_pose = Pose()
                place_pose = Pose()

                # name
                object_name.data = pick_object['name']

                # pickpose
                # Get the PointCloud for a given object and obtain it's centroid
                points_arr = ros_to_pcl(object.cloud).to_array()
                tmp =  np.mean(points_arr, axis=0)[:3]
                pick_pose.position.x = np.asscalar(tmp[0])
                pick_pose.position.y = np.asscalar(tmp[1])
                pick_pose.position.z = np.asscalar(tmp[2])

                labels.append(object.label)
                centroids.append(tmp)

                # based on group (green or red) place object in left or right bins
                object_group = pick_object['group']
                arm_name.data = drop_position[object_group][0]

                position = drop_position[object_group][1]
                eps1 = 0.01
                new_pos = [p + eps1*np.random.rand() for p in position]

                place_pose.position.x = new_pos[0]
                place_pose.position.y = new_pos[1]
                place_pose.position.z = new_pos[2]

                yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose,
                                           place_pose)
                dict_list.append(yaml_dict)


    yaml_filename = '/home/robond/catkin_ws/output_{}.yaml'.format(test_scene_num.data)
    send_to_yaml(yaml_filename, dict_list)

    # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format


if __name__ == '__main__':

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
    model_dir =  '../../data/train/'
    version=7; nbins=64; samples=500; kernel_type='linear'; cost=10
    model_file = 'model{}_samples{}_nbins{}_kernel_{}_C_{}.sav'.format(version, samples, nbins,  kernel_type, cost)
    full_model_name = '{}/{}'.format(model_dir,model_file)
    model = pickle.load(open(full_model_name, 'rb'))
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
