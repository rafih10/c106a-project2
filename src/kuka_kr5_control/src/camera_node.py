#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy

# Import for camera image
from sensor_msgs.msg import Image

# Type used to publish the location of the ball
from nav_msgs.msg import Odometry

# Imports for math stuff
import numpy as np
import cv2


# Subscribes to camera inputs 
def camera_listener():
    # Topic = "/my_camera/sensor/camera/rgb/image_raw"
    camera_subscriber = rospy.Subscriber("/my_camera/sensor/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()


# Takes the information from the image and clusters it
def image_callback(ros_image):
    # First process the image so its useable to cv package
    cv2_image = convert_image(ros_image)

    # Cluster the converted image so that the ball is in one cluster
    clustered_image = cluster_image(cv2_image)

    # Find the cluster with the ball in it 
    ball_cluster = get_ball_cluster(clustered_image)

    # Take the ball cluster and convert it to usable coordinates
    ball_coordinates = get_ball_coordinates(ball_cluster)

    # Convert the coordinates to an odometry message which can be published to plate_control
    ball_odom_coords = convert_to_odom(ball_coordinates)

    # Publish the coordinates
    ball_pub.publish(ball_odom_coords)


# Converts the image from sensor_msgs.msg -> some usable form by cv2 
# Returns converted image
def convert_image(ros_image):
    # To Do Here
    return 1


# Clusters the cv2 image and returns the clustered version
def cluster_image(cv2_image):
    # To Do Here
    return 1


# Given a clustered image, will find the ball and returns that cluster
def get_ball_cluster(clustered_image):
    # To Do Here
    return 1


# Given the cluster that the ball is in, will return the coordinates of the ball
# in the Gazebo simulation
def get_ball_coordinates(cluster):
    # To Do Here
    return 1


# Converts the coordinates of the ball to a message which can be published to 
# the cpp plate_control_node
def convert_to_odom(coordinates):
    # To Do here
    return 1


# If this file is started
if __name__ == '__main__':
    # Publishes the location of the ball
    ball_pub = rospy.Publisher('ball_coordinates', Odometry, queue_size=10)

    # Start listening to the camer node
    camera_listener()