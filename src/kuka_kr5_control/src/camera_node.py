#!/usr/bin/env python

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy

# Import for camera image
from sensor_msgs.msg import Image

# Type used to publish the location of the ball
from nav_msgs.msg import Odometry

# Used to convert the rospy image to something usable by cv2
from cv_bridge import CvBridge, CvBridgeError

# Used for making the odometry points
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Used to create the quaternion
import tf

# Imports for math stuff
import numpy as np
import cv2


# Subscribes to camera inputs 
def camera_listener():
    # Topic = "/my_camera/sensor/camera/rgb/image_raw" - camera information
    camera_subscriber = rospy.Subscriber("/my_camera/sensor/camera/rgb/image_raw", Image, image_callback)

    # Wait until this node start getting information
    rospy.spin()


# Takes the information from the image and clusters it
def image_callback(ros_image):
    # First process the image so its useable by the cv package a
    cv2_image = convert_image(ros_image)

    # # Checking to make sure the image was properly converted
    # if (cv2_image == -1):
    #     # Stop processing since there was an error
    #     return

    # Cluster the converted image so that the ball is in one cluster
    clustered_image = cluster_image(cv2_image)

    # Wait for a bit of time
    cv2.imshow('test_window', clustered_image)
    cv2.waitKey(20)
    return 

    # Get the ball coordinates
    x_coord, y_coord = get_ball_coordinates(clustered_image)

    # Convert the coordinates to an odometry message which can be published to plate_control
    ball_odom = convert_to_odom(x_coord, y_coord)

    # Publish the coordinates
    ball_pub.publish(ball_odom)


# Converts the image from sensor_msgs.msg -> some usable form by cv2 
# Returns converted image
def convert_image(ros_image):
    try:
        # Try to convert the image and return the converted image in rbg
        return bridge.imgmsg_to_cv2(ros_image, "rgb8")
    except CvBridgeError as e:
        # Print our error and return -1
        print(e)
        return -1

# Clusters the cv2 image and returns the clustered version
def cluster_image(img, n_clusters=3, random_state=0):
    # Reshaping the image into a 2D array of pixels and 3 color values (RGB) and converting to float type
    img_r = np.float32(img.reshape((-1,3)))

    # Perfrom k means clustering
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = n_clusters
    ret, label, center = cv2.kmeans(img_r, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    # convert the image back to 8 bit values
    center = np.uint8(center)
    res = center[label.flatten()]

    # Print out the array of centers of the clusters
    print(center)

    # Return the reshaped image
    return res.reshape((img.shape))


# Given a clustered image, will find the ball and returns that cluster
def get_ball_coordinates(clustered_image):
    # Search through clustered image for ball
    return 0, 1


# Converts the coordinates of the ball to a message which can be published to 
# the cpp plate_control_node
def convert_to_odom(x_coord, y_coord):
    # Global variables used for velocity calculation
    global last_time, previous_x, previous_y

    # Get the current_time
    current_time = rospy.Time.now()

    # Create the Odom message
    odom = Odometry()

    # Set the header of the odom message
    odom.header.stamp = current_time    # Set the timestamp for the odom
    odom.header.frame_id = "ee_link"    # Found this in plate_control_node, not sure if it is right or not

    # Converting the coords on the camera to actual coordinates
    x_coord = x_coord * x_mul_constant + x_add_constant
    y_coord = y_coord * y_mul_constant + y_add_constant

    # set the position
    odom.pose.pose = Pose(Point(x_coord, y_coord, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "idk"
    
    # Calculate the velocity using a frame by frame calculation
    dt = current_time - last_time
    vx = (x_coord - previous_x) / dt
    vy = (y_coord - previous_y) / dt

    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, 0))

    # Set the last values
    last_time = current_time
    previous_x = x_coord
    previous_y = y_coord

    # To Do here
    return odom
    

# If this file is started
if __name__ == '__main__':
    # Create the node/publisher to the ball_coordinates node
    rospy.init_node('ball_coordinates')
    ball_pub = rospy.Publisher('ball_coordinates', Odometry, queue_size=10)

    # Need to save these variables for odom calculations
    last_time = 0
    previous_x = 0
    previous_y = 0

    # Used to convert from camera coords to irl coords
    x_mul_constant = 1
    y_mul_constant = 1
    x_add_constant = 0
    y_add_constant = 0

    # Bridges between rospy image and cv2 image
    bridge = CvBridge()

    # Quaternion is not used, so creating a default one for the pose
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    # Start listening to the camer node
    camera_listener()