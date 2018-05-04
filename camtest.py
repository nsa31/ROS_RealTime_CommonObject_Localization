import message_filters
from sensor_msgs.msg import Image, CameraInfo
import rospy, cv2, cv_bridge

def callback(image, camera_info):
  # Solve all of perception here...
  print 'sth'
  img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
  cv2.imshow("result", img)
  cv2.waitKey(3)

rospy.init_node('prj')	
image_sub = message_filters.Subscriber('camera/rgb/image_raw', Image)
info_sub = message_filters.Subscriber('camera/depth_registered/image_raw', CameraInfo)

ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
ts.registerCallback(callback)
rospy.spin()
