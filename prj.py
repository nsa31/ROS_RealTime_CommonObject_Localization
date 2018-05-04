#!/usr/bin/env python

import rospy, cv2, cv_bridge
import actionlib
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import math
import thread
import imutils
#pip install playsound
from playsound import playsound
import os
#for sockets
import socket
import sys
import pickle
import struct ### new code
from matplotlib import pyplot as plt
from threading import Thread
import pyttsx #sudo pip install pyttsx


global waypoints

waypoints = [
    #[(1.138, 0.924, 0.0), (0.0, 0.0,  -0.998, 0.0482)],
    [(-0.481, 1.711, 0.0), (0.0, 0.0, 0.929,  0.369)], 
    [(0.289, -0.629, 0.0), (0.0, 0.0,  -0.907, 0.419)], 
    [(2.209, -0.784, 0.0), (0.0, 0.0, -0.325, 0.945)],
    [(2.449, 2.004, 0.0), (0.0, 0.0,  0.448, 0.893)],
    [(1.354, -0.328, 0.0), (0.0, 0.0,  -0.657, 0.753)],
    [(-0.481, 1.711, 0.0), (0.0, 0.0, 0.929,  0.369)], 
    [(0.289, -0.629, 0.0), (0.0, 0.0,  -0.907, 0.419)], 
    [(2.209, -0.784, 0.0), (0.0, 0.0, -0.325, 0.945)],
    [(2.449, 2.004, 0.0), (0.0, 0.0,  0.448, 0.893)],
    [(1.354, -0.328, 0.0), (0.0, 0.0,  -0.657, 0.753)]
]
global CLASSES
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
global COLORS
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

class Prj:
	def __init__(self):

		self.idx = 0
		self.startX = 0
		self.endX = 0
		self.startY = 0
		self.endY = 0
		self.engine = pyttsx.init()
		
		self.state= 1 # state 0 idle , state 1 search , state 2 follow
		
		while True:
			try:
				ip = '0.0.0.0' #raw_input('Sever IP:')
				port = 8002 #int(raw_input('Port:'))
				self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				self.s.connect((ip, port))
			except socket.error as error:
				print('Error while connecting')
				print(error)
				print('')
			else:
				break
		
		self.imgMap=cv2.imread('mymap2.png')
		self.bridge = cv_bridge.CvBridge()
		
		self.amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)
		self.caminfo_sub = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.caminfo_cb)
		self.img_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_cb)
                #self.Dimg_sub = rospy.Subscriber('camera/depth_registered/image_raw', Image, self.Dimage_cb) #works sometimes
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
		#self.cmd_vel_pub = rospy.Publisher('cmd_vel/velocityramp', Twist, queue_size=1)
		self.joy = rospy.Subscriber('/joy', Joy, self.joy_cb)

		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
		self.twist = Twist()
		self.twist.linear.x = 0.1
		self.twist.angular.z = 0.1
		
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()
		
		self.move = False
                self.loop_exit_check = False
		self.range_ahead = 0
		self.depth = []
                self.PI = 3.1416
		self.rx=0
		self.ry=0
		self.ox=0
		self.oy=0
		self.arc=0
		self.angle=0
		self.cx=0
		self.cy=0

		self.time_loc = time.time() + 15.00 #15 sec to localize 
		self.service = rospy.ServiceProxy('global_localization', Empty)
		self.service()		
		

	def makeSound(self , i=1):
		if(i==1) :
			playsound('crash.wav')
		else :
			playsound('bell.wav')

	def sendData(self,sock, data):
		'''
		Send string through socket.
		'''
		encoded_data = data.encode('utf-8')
		sock.send(struct.pack('Q', len(encoded_data)))
		sock.send(bytes(encoded_data)) 

	def receiveData(self,sock):
		'''
		Receive object from socket.
		'''
		lengthLeft = struct.unpack('Q', sock.recv(struct.calcsize('Q')))[0]
		data = ''
		while lengthLeft > 0:
			block = sock.recv(lengthLeft)
			data += block
			lengthLeft -= len(block)
		return data.decode('utf-8')

	def getBBox(self):
		while True :
			#print 'ok'
			self.idx= int(self.receiveData(self.s))
			#print(self.idx)
			self.startX=int(self.receiveData(self.s))
			#print(self.startX)
			self.startY= int(self.receiveData(self.s))
			#print(self.startY)
			self.endX= int(self.receiveData(self.s))
			#print(self.endX)    
			self.endY= int(self.receiveData(self.s))
			#print(self.endY)
			if self.idx == 7 : #car follower
				if self.state == 1 :
					self.engine.say('Car Detected at')
					self.engine.runAndWait()
				dis = self.find_range_depth(self.startX,self.endX)
				dis = int(dis*100)
				print('distance',dis)
				#self.engine.say(str(int(dis)))
				#self.engine.runAndWait()
				self.state=2
				self.client.cancel_goal()
				print('cancelling the goal')
				self.cx = (self.startX+self.endX)/2
				self.cy = self.endY
				err = self.cx - 320 #640/2
				if dis < 15 :
					self.twist.linear.x = 0
				else :
					self.twist.linear.x = 0.2
				kp = float(-1.0/100) *err
				print err,-float(err) / 100 , kp
				self.twist.angular.z = -float(err) / 1000 *2
				self.cmd_vel_pub.publish(self.twist)
				#break
			if self.idx == 5 : #bottle plot
				self.arc =  self.find_range_depth(self.startX,self.endX)
			else :
				dis=0

	def bbox_threaded(self): #use thread within a class to get bounding box from another python program
		Thread(target=self.getBBox).start()

	def amcl_cb(self, msg):
		#print msg.pose.pose.position.x
		#print msg.pose.pose.position.y
		tx=float(msg.pose.pose.position.x)
		ty=float(msg.pose.pose.position.y)
		tz=float(msg.pose.pose.position.z) 
		
		tox=float(msg.pose.pose.orientation.x)
		toy=float(msg.pose.pose.orientation.y)
		toz=float(msg.pose.pose.orientation.z)
		tow=float(msg.pose.pose.orientation.w)

		orientation_list = [tox, toy, toz, tow] 
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.angle=yaw
		print('yaw') 
		print(yaw)
		
		self.rx= int(145 +tx*70)
		self.ry= int(515 - ty*70)

		#y= 515#int(674-(515 + 5.03*70))
		#print self.rx,self.ry
		#self.imgMap=cv2.imread('mymap2.png')
		#plt.imshow(self.imgMap)
		#plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
		#plt.show()
	
	def image_cb(self, msg):
		global CLASSES , COLORS
		img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		#cv2.rectangle(img, (startX, startY), (endX, endY), (0, 0, 255), 2)
    
		label = "{}".format(CLASSES[self.idx])
		cv2.rectangle(img, (self.startX, self.startY), (self.endX, self.endY),COLORS[self.idx], 2)
		y = self.startY - 15 if self.startY - 15 > 15 else self.startY + 15
		cv2.putText(img, label, (self.startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[self.idx], 2)
		cv2.circle(img, (self.cx, self.cy), 20, (0,0,255), -1)
		cv2.imshow("result", img)
		
		self.imgMap=cv2.imread('mymap2.png')
		cv2.circle(self.imgMap, (self.rx, self.ry), 5, (255,255,0), -1)
		print('arc',self.arc)
		print('ang',self.angle)
		tx= self.rx + self.arc*100*math.cos((self.angle*3)/180*3.1416)
		ty= self.ry + self.arc*100*math.sin((self.angle*3)/180*3.1416)
		cv2.circle(self.imgMap, (int(tx), int(ty)), 5, (255,0,0), -1)
		cv2.imshow("Frame2", self.imgMap)
		cv2.waitKey(3)
		#cv2.waitKey(3)
		#print self.imgMap.shape
		

	def Dimage_cb(self,msg_depth):
		cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")

		cv2.imshow("Image from my node", cv_image)
		cv2.waitKey(1)	

	def caminfo_cb(self, msg):
		self.K = np.array(msg.K).reshape(3,3)
		self.D = np.array(msg.D)
	
	def scan_cb(self, msg):
		
		#print 'scn'
		#print len(msg.ranges)
		self.depth=np.zeros(641)+999
		for c ,dist in enumerate(msg.ranges):
			if not np.isnan(dist):
				self.depth[c]=dist
			else :
				self.depth[c]= 999
		if len(self.depth) == 0:
			self.range_ahead = 0.8
		else:
			self.range_ahead = np.min(self.depth)

		#print self.range_ahead
		#print len(self.depth)
		#print(self.depth[200:210])
		#print(min(self.depth[200:210]))
		#print(self.find_range_depth(200,210))
		

	def find_range_depth(self,x1,x2) : #find the depth from the length of bounding box
		if x1>=0 and x2<640 :
			return min(self.depth[x1:x2])
		else :
			return 0.8

	def joy_cb(self, msg):
		if msg.buttons[0]:
			self.move = not self.move
                        print "move flag"
                elif msg.buttons[1]==1:
                        print 'Exit...'  
                        self.loop_exit_check = not self.loop_exit_check
                        self.client.cancel_goal()
                        print "cancel"
                        #os._exit(0)
			
	
	def rotate(self, speed , angle , clockwise): #generalized function to rorate 
		#Converting from angles to radians
		while not self.move:
                        print "Waitning for joystick"
                        pass
		angular_speed = speed
		relative_angle = angle*2.0*self.PI/360

		#We wont use linear components
		self.twist.linear.x=0
		self.twist.linear.y=0
		self.twist.linear.z=0
		self.twist.angular.x = 0
		self.twist.angular.y = 0

    # Checking if our movement is CW or CCW
                print angular_speed , '###########'
		if clockwise:
			self.twist.angular.z = -abs(angular_speed)
		else:
			self.twist.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
               
		while(current_angle < relative_angle):
                        if self.loop_exit_check:
		                break
			self.cmd_vel_pub.publish(self.twist)
			t1 = rospy.Time.now().to_sec()
		        current_angle = angular_speed*(t1-t0)
                        #print current_angle
        
	def moveForward(self, speed, distance, isForward):  #generalized funtion to move certain linear distance
		if(isForward):
			self.twist.linear.x = abs(speed)
		else:
			self.twist.linear.x = -abs(speed)
    #Since we are moving just in x-axis
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0

    #Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_distance = 0

    #Loop to move the turtle in an specified distance
		while(current_distance < distance):
      #Publish the velocity
                        if self.loop_exit_check:
		                break
			self.cmd_vel_pub.publish(self.twist)
			#Takes actual time to velocity calculus
			t1=rospy.Time.now().to_sec()
			#Calculates distancePoseStamped
			current_distance= speed*(t1-t0)
		#After the loop, stops the robot
		self.twist.linear.x = 0
		#Force the robot to stop
		self.cmd_vel_pub.publish(self.twist)
		#print "END"
	
	def goal_pose(self,pose):
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.pose.position.x = pose[0][0]
		goal_pose.target_pose.pose.position.y = pose[0][1]
		goal_pose.target_pose.pose.position.z = pose[0][2]
		goal_pose.target_pose.pose.orientation.x = pose[1][0]
		goal_pose.target_pose.pose.orientation.y = pose[1][1]
		goal_pose.target_pose.pose.orientation.z = pose[1][2]
		goal_pose.target_pose.pose.orientation.w = pose[1][3]
		return goal_pose
    
	def mainAlgo(self):
		
		while True:
			
			if rcamx != -1:
				#rotate(0.5, 90, 1)
				print "FOUND" 
			else:
				print "NONE"

	
	def searchGoals(self): 
		i = 1  	
		global waypoints
		for pose in waypoints:
			if self.state!=1:
		                #break
				return
			curr_pose = pose
			goal = self.goal_pose(pose)
			print 'going . . .'
			print i
			i = i + 1
			self.client.send_goal(goal)
			
			self.client.wait_for_result()
			print "REACHED GOAL"
			
                        if self.loop_exit_check:
		                break

		
	def moveLocalize(self): #can be used for initial localization
                while not self.move:
                        print "WHILE"
                        pass
		while time.time() < self.time_loc:
			print time.time()
		
			if self.range_ahead > 2.0:
				self.twist.angular.z = (self.twist.angular.z + math.sin(time.time()/2.0)/2.0)/2.0
				self.twist.linear.x = (4.0 * self.twist.linear.x + 0.5) / 5.0
				self.cmd_vel_pub.publish(self.twist)
			else:
				self.twist.angular.z = (self.twist.angular.z + 0.5) / 2.0
				self.twist.linear.x = (self.twist.linear.x + 0.0) / 2.0
				self.cmd_vel_pub.publish(self.twist)
			#rate = rospy.Rate(500)
			#global errx, depth
                        if self.loop_exit_check:
		                break
		print "FINISH WHILE"
  	
    
if __name__ == "__main__":
	rospy.init_node('prj')		
	print "START"	
	prj = Prj()
	prj.bbox_threaded()

        #prj.moveLocalize()
	prj.rotate(0.4, 360, 1)
	prj.searchGoals()
  	print "end"
	rospy.spin()

# END ALL









	
