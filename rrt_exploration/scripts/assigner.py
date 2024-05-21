#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import OccupancyGrid
#import tf
import tf2_geometry_msgs
import tf2_kdl
import tf2_msgs
import tf2_py
import tf2_ros
import tf2_sensor_msgs
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount,gridValue,unvalid
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
#---wrote myself---
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Pose2D
import time
from numpy import delete,where
count=0
bum=rospy.Publisher( '/robot_1/mobile_base/commands/velocity', Twist, queue_size = 10 )
#------------------
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
#--wrote myself--
def bumpercallback(bumper):
	vel=Twist()
	vel.linear.x = -0.5
	rate=rospy.Rate(10.0)
	for i in range(5):
		bum.publish(vel)
		rate.sleep()
#----------------
# Node----------------------------------------------

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps,bum,count
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','map')
	info_radius= rospy.get_param('~info_radius',1.0)#default 1.0					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)#default 3.0	
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)#default 3.0			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)#default 2.0				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','filtered_points')	
	n_robots = rospy.get_param('~n_robots',1)#default 1
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count',1)#default 1
	delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)#default 0.5
	rateHz = rospy.get_param('~rate',100)#default 100
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
	rospy.Subscriber( '/robot_1/mobile_base/events/bumper', BumperEvent, bumpercallback)
	#---wrote myself---
	"""
	truepub = rospy.Publisher('trueCentroids', Marker, queue_size=10)
	trueCentroid=Marker()
	trueCentroid.header.frame_id = mapData.header.frame_id
	trueCentroid.header.stamp = rospy.Time.now()
	trueCentroid.ns = "markers4"
	trueCentroid.id = 5
	trueCentroid.type = Marker.POINTS
	trueCentroid.action = Marker.ADD
	trueCentroid.pose.orientation.w = 1.0
	trueCentroid.scale.x = 0.2
	trueCentroid.scale.y = 0.2
	trueCentroid.color.r = 255.0/255.0
	trueCentroid.color.g = 0.0/255.0
	trueCentroid.color.b = 255.0/255.0
	trueCentroid.color.a = 1
	trueCentroid.lifetime = rospy.Duration()
	p=Point()
	p.z=0
	"""
	#------------------
#---------------------------------------------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots=[]
	if len(namespace)>0:
		for i in range(0,n_robots):
			robots.append(robot(namespace+str(i+namespace_init_count)))
	elif len(namespace)==0:
			robots.append(robot(namespace))
	for i in range(0,n_robots):
		robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	while not rospy.is_shutdown():
		centroids=copy(frontiers)	
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain=[]
		for ip in range(0,len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na=[] #available robots
		nb=[] #busy robots
		for i in range(0,n_robots):
			if (robots[i].getState()==1):
				nb.append(i)
			else:
				na.append(i)	
		#rospy.loginfo("available robots: "+str(na))	
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
		revenue_record=[]
		centroid_record=[]
		id_record=[]
		

		for ir in na:
			for ip in range(0,len(centroids)):
				#---wrote myself---
				#if(unvalid(mapData,centroids[ip])):
					#print ('cancel frontier')

					#continue
				if(unvalid(mapData,[centroids[ip][0],centroids[ip][1]])):
					continue
				#------------------
				cost=norm(robots[ir].getPosition()-centroids[ip])		
				threshold=1
		
				information_gain=infoGain[ip]
				#---wrote myself---
				#"""
				distance=norm(robots[ir].getPosition()-centroids[ip])
				if (distance<=hysteresis_radius):
					information_gain*=(hysteresis_gain)*(hysteresis_radius/distance)
				#"""
				#-------------
				"""
				distance=norm(robots[ir].getPosition()-centroids[ip])
				if (distance<=hysteresis_radius):
					information_gain*=(hysteresis_gain+1-distance)
				#"""
				#------------------default
				"""
				if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
					information_gain*=hysteresis_gain
				#"""
				revenue=information_gain*info_multiplier-cost
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])
				id_record.append(ir)
		
		if len(na)<1:
			revenue_record=[]
			centroid_record=[]
			id_record=[]
			for ir in nb:
				for ip in range(0,len(centroids)):
					#--wrote myself---
					if(unvalid(mapData,[centroids[ip][0],centroids[ip][1]])):
						continue
					#-----------------
					cost=norm(robots[ir].getPosition()-centroids[ip])		
					threshold=1
					information_gain=infoGain[ip]

					#---wrote myself---
					#"""
					distance=norm(robots[ir].getPosition()-centroids[ip])
					if (distance<=hysteresis_radius):
						information_gain*=(hysteresis_gain)*(hysteresis_radius/distance)

					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)
						information_gain*=(hysteresis_gain)*(hysteresis_radius/distance)
					#"""
					#-----------
					"""
					distance=norm(robots[ir].getPosition()-centroids[ip])
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=(hysteresis_gain+1-distance)
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*(hysteresis_gain+1-distance)
					#"""
					#------------------default
					"""
					if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
						information_gain*=hysteresis_gain
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain
					#"""
					revenue=information_gain*info_multiplier-cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		#rospy.loginfo("revenue record: "+str(revenue_record))	
		#rospy.loginfo("centroid record: "+str(centroid_record))	
		#rospy.loginfo("robot IDs record: "+str(id_record))	
		#---wrote myself---
		"""
			pp=[]
			for q in range(0,len(centroid_record)):
				p.x=centroid_record[q][0]
				p.y=centroid_record[q][1]
				pp.append(copy(p))
			trueCentroid.points=pp
			truepub.publish(trueCentroid)
		"""
		#------------------
#-------------------------------------------------------------------------	
		if (len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
			#rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))
			#rospy.loginfo(unvalid(mapData,[centroid_record[winner_id][0],centroid_record[winner_id][1]]))
			count=0
			rospy.sleep(delay_after_assignement)
		#---wrote myself---
		
		else:
			count+=1
			#if(count>=100):
			#	rospy.loginfo(count)
			if(count>=3000):
				rospy.loginfo('completed')
				sys.exit()
		"""
		while(len(id_record)>0):
			winner_id=revenue_record.index(max(revenue_record))
			if(unvalid(mapData,[centroid_record[winner_id][0],centroid_record[winner_id][1]])):
				#centroid_record = delete(centroid_record, winner_id, axis=0)
				#revenue_record = delete(revenue_record, winner_id, axis=0)
				#id_record = delete(id_record, winner_id, axis=0)
				del centroid_record[winner_id]
				del revenue_record[winner_id]
				del id_record[winner_id]
			else:
				robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
				rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))
				rospy.sleep(delay_after_assignement)
				break
		#"""
		#------------------
	
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
