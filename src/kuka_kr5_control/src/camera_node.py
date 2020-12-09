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
import matplotlib.pyplot as plt


# Subscribes to camera inputs 
def camera_listener():
    # Topic = "/my_camera/sensor/camera/rgb/image_raw"
    camera_subscriber = rospy.Subscriber("/my_camera/sensor/camera/rgb/image_raw", Image, image_callback)

    rospy.spin()


# Takes the information from the image and clusters it
def image_callback(image):
    pass


# If this file is started
if __name__ == '__main__':
    # Publishes the location of the ball
    pub = rospy.Publisher('ball_coordinates', Odometry, queue_size=10)

    # Start listening to the camer node
    camera_listener()