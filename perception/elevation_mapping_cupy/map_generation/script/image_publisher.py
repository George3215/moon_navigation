#!/usr/bin/env python
# simple script to publish a image from a file.
import rospy
import rospkg
import time
import cv2
import sensor_msgs.msg
import numpy as np
from std_msgs.msg import Bool

def callback(self):
    """ Convert a image to a ROS compatible message
        (sensor_msgs.Image).
    """
    global publisher, imagePath
    img = cv2.imread(imagePath, cv2.IMREAD_UNCHANGED)
    rosimage = sensor_msgs.msg.Image()

#    print img.shape
#    print img.size
#    print img.dtype.itemsize

    # rosimage = sensor_msgs.msg.Image()
    if img.dtype.itemsize == 2:
       if len(img.shape) == 3:
           if img.shape[2] == 3:
               rosimage.encoding = 'bgr16'
           if img.shape[2] == 4:
           	   rosimage.encoding = 'bgra16'
       else:
           rosimage.encoding = 'mono16'
    if img.dtype.itemsize == 1:
       if len(img.shape) == 3:
           if img.shape[2] == 3:
               rosimage.encoding = 'bgr8'
           if img.shape[2] == 4:
           	   rosimage.encoding = 'bgra8'
       else:
           rosimage.encoding = 'mono8'
           #print ("Encoding: ", rosimage.encoding)
           rosimage.encoding = 'bgr8'
           img = np.stack((img,)*3, axis=-1)
           #print("transform ok") 

    #print ("Encoding: ", rosimage.encoding)

    rosimage.width = img.shape[1]
    rosimage.height = img.shape[0]
    rosimage.step = img.strides[0]
    rosimage.data = img.tostring()
    rosimage.header.stamp = rospy.Time.now()
    rosimage.header.frame_id = map_frame

    publisher.publish(rosimage)

def ShutdownCallback(msg):
    if msg.data:
        rospy.signal_shutdown("Shutdown requested")
    else:
        print("Shutdown request received, but ignored")
        pass


#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publisher, imagePath,map_frame
    rospack = rospkg.RosPack()
    rospy.init_node('image_publisher')
    # imagePath = "../maps/resized/HM17.png"
    # topicName = "~image"
    imagePath = rospy.get_param('~image_path')
    topicName = rospy.get_param('~topic')
    #print(imagePath)
    map_frame = rospy.get_param('~map_frame_id')
    #print(map_frame)
    shutdown_sub = rospy.Subscriber("/mode1/shutdown_request", Bool, ShutdownCallback)
    publisher = rospy.Publisher(topicName, sensor_msgs.msg.Image, queue_size=10)
    rospy.Timer(rospy.Duration(2), callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
