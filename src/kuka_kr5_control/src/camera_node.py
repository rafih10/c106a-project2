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

    ball_subscriber = rospy.Subscriber("/gazebo/ball/odom", Odometry, ball_callback)

    # Wait until this node start getting information
    rospy.spin()

def ball_callback(ball_coord):
	ball_point = ball_coord.pose.pose.position
	print("Actual ball: "+ "X: " + str(ball_point.x) + " Y: " + str(ball_point.y))


# Takes the information from the image and clusters it
def image_callback(ros_image):
    # First process the image so its useable by the cv package a
    cv2_image = convert_image(ros_image)

    # # Checking to make sure the image was properly converted
    # if (cv2_image == -1):
    #     # Stop processing since there was an error
    #     return

    # Cluster the converted image so that the ball is in one cluster
    #clustered_image = cluster_image(cv2_image)
    cv2.imshow("test",cv2_image)
    cv2.waitKey(10)

    # Get the ball coordinates using the clustered image
    #x_coord, y_coord = get_ball_coordinates(clustered_image)

    # Get the ball coordinates using the regular image and looking for yellow
    y_coord, x_coord = get_ball_coordinates(cv2_image)
    if (y_coord == -1 or x_coord == -1):
    	return
    # return

    # Convert the coordinates to an odometry message which can be published to plate_control
    ball_odom, x_coord, y_coord = convert_to_odom(x_coord, y_coord)

    # Publish the coordinates
    ball_pub.publish(ball_odom)
    print("X: " + str(x_coord) + " Y: " + str(y_coord))


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
def cluster_image(img, n_clusters=4, random_state=0):
    # Reshaping the image into a 2D array of pixels and 3 color values (RGB) and converting to float type
    img_r = np.float32(img.reshape((-1,3)))

    # Perfrom k means clustering
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = n_clusters
    ret, label, center = cv2.kmeans(img_r, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    # convert the image back to 8 bit values
    center = np.uint8(center)
    res = center[label.flatten()]

    # Return the reshaped image
    return res.reshape((img.shape))


# Given a clustered image, will find the ball and returns that cluster
def get_ball_coordinates(clustered_image):
    if (camera_vals[0]):
        # Search through clustered image for ball
        camera_x = camera_vals[1]
        camera_y = camera_vals[2]
        for x in range(camera_x - 30, camera_x + 30):
            row = clustered_image[x]
            for y in range(camera_y - 30, camera_y + 30):
                # Check if any of the pixels are not white
                if (row[y][0] < 100):
                    camera_vals[1] = x
                    camera_vals[2] = y
                    return x, y
    # Search through clustered image for ball
    for x in range(80, 300):
        row = clustered_image[x]
        for y in range(150, 500):
            # Check if any of the pixels are not white
            if (row[y][0] < 100):
                camera_vals[0] = True
                camera_vals[1] = x
                camera_vals[2] = y
                return x, y
    # Ball is off the table
    print(camera_vals[0])
    camera_vals[0] = False
    return -1, -1


# Converts the coordinates of the ball to a message which can be published to 
# the cpp plate_control_node
def convert_to_odom(x_coord, y_coord):
    # Global variables used for velocity calculation
    last_time, previous_x, previous_y = prev_vals

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
    dt = 0.1 #current_time - last_time
    vx = (x_coord - previous_x) / dt
    vy = (y_coord - previous_y) / dt

    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, 0))

    # Set the last values
    prev_vals[0] = current_time
    prev_vals[1] = x_coord
    prev_vals[2] = y_coord

    # To Do here
    return odom, x_coord, y_coord
    

# If this file is started
if __name__ == '__main__':
    # Create the node/publisher to the ball_coordinates node
    rospy.init_node('ball_coordinates')
    ball_pub = rospy.Publisher('ball_coordinates', Odometry, queue_size=10)

    # Need to save these variables for odom calculations
    last_time = 0
    previous_x = 0
    previous_y = 0
    prev_vals = [0, 0, 0]

    # Used to convert from camera coords to irl coords
    x_mul_constant = -0.0018
    y_mul_constant = -0.00068
    x_add_constant = 1.6
    y_add_constant = 0.26012

    # Using these variables to make the camera faster
    flipped_plate = False
    camera_x = -1
    camera_y = -1

    camera_vals = [flipped_plate, camera_x, camera_y]

    # Bridges between rospy image and cv2 image
    bridge = CvBridge()

    # Quaternion is not used, so creating a default one for the pose
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    # Start listening to the camer node
    camera_listener()