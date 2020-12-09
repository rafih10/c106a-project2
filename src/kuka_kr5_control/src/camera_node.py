#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy

# Import for camera image
from sensor_msgs.msg import Image

# Type used to publish the location of the ball
from nav_msgs.msg import Odometry

# Used to convert the rospy image to something usable by cv2
from cv_bridge import CvBridge, CvBridgeError

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

    # Checking to make sure the image was properly converted
    if (cv2_image == -1):
        # Stop processing since there was an error
        return

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
    try:
        # Try to convert the image and return the converted image in grayscale
        return bridge.imgmsg_to_cv2(ros_image, "mono8")
    except CvBridgeError as e:
        # Print our error and return -1
        print(e)
        return -1


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
    # Create the node/publisher to the ball_coordinates node
    rospy.init_node('ball_coordinates')
    ball_pub = rospy.Publisher('ball_coordinates', Odometry, queue_size=10)

    # Bridges between rospy image and cv2 image
    bridge = CvBridge()

    # Start listening to the camer node
    camera_listener()