#!/usr/bin/env python3

# Standard imports
import os, sys, time
import threading

# ROS imports
import rospy

# Custom BPL RS2 ROS messages
from blueprintlab_reachsystem_ros_messages.msg import single_float
from blueprintlab_reachsystem_ros_messages.msg import single_int
from blueprintlab_reachsystem_ros_messages.msg import generic
from blueprintlab_reachsystem_ros_messages.msg import request_list

from RS1_hardware import PacketID
from packetid import Packets

FLOAT_PACKETS = Packets().get_float_packet_ids()

'''
Reach5Mini Controller Class - ability to subscribe and publish to the Reach5Mini passthrough scripts
Arguents:
	r5m_instance (optional)(defaults to '0') - Instance number of the Reach5Mini
	device_ids (optional)(defaults to [1,2,3,4,5]) - Device IDs associated with the particular Reach5Mini

'''
class Reach5MiniROSController():
	instances = []

	def __init__(self, r5m_instance=0, device_ids=[1,2,3,4,5], heartbreat_frequency=20, request_timeout=0.1, **kwargs):
		self.r5m_instance = r5m_instance
		self.device_ids = 	device_ids
		self.wait_for_heatbeat = False
		self.heartbreat_frequency = heartbreat_frequency
		self.request_timeout = request_timeout

		for r5m_inst in Reach5MiniROSController.instances:
			if r5m_inst.r5m_instance == self.r5m_instance:
				raise Exception('Cannot use the same r5m_instance number as other Reach5MiniROSController class instances!')
		Reach5MiniROSController.instances.append(self)		

		# Init subscribers and publishers
		self.cmd_velocity_pub = rospy.Publisher('r5m_' + str(self.r5m_instance) + '/cmd_velocity', single_float, queue_size=10)
		self.cmd_position_pub = rospy.Publisher('r5m_' + str(self.r5m_instance) + '/cmd_position', single_float, queue_size=10)
		self.requests_pub = 	rospy.Publisher('r5m_' + str(self.r5m_instance) + '/requests', request_list, queue_size=10)
		self.position_sub = 	rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/position', single_float, self.position_callback)        
		self.velocity_sub = 	rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/velocity', single_float, self.velocity_callback)
		self.current_sub = 		rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/current', single_float, self.current_callback)
		self.openloop_sub = 	rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/openloop', single_float, self.openloop_callback)
		self.mode_sub = 		rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/mode', single_int, self.mode_callback)  
		self.other_callback = 	rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/other', generic, self.other_callback)      

		# Define messages
		self.cmd_velocity_msg = single_float()
		self.position_msg =		single_float()
		self.velocity_msg = 	single_float()
		self.position_msg = 	single_float()
		self.current_msg =		single_float()
		self.mode_msg =			single_int()
		self.other_msg =		generic()
		self.requests_msg =		request_list()

	'''
	Main Script - used to create necessary threads
	'''
	def start(self):
		self.thread_request_loop = threading.Thread(target=self.heartbeat_request)
		self.thread_request_loop.setDaemon(True)
		self.thread_request_loop.start()

	'''
	Command and Control Functions	
	'''	
	def send_velocity_command(self, device_id, velocity):
		if device_id in self.device_ids and velocity is not None:
			self.cmd_velocity_msg.stamp = 		rospy.get_rostime()
			self.cmd_velocity_msg.device_id = 	device_id
			self.cmd_velocity_msg.value =		velocity
			self.cmd_velocity_pub.publish(self.cmd_velocity_msg)
		else:
			print(__name__, 'send_velocity_command():', 'Cannot send NoneType value!')

	def send_position_command(self, device_id, position):
		if device_id in self.device_ids and position is not None:
			self.cmd_position_msg.stamp = 		rospy.get_rostime()
			self.cmd_position_msg.device_id = 	device_id
			self.cmd_position_msg.value =		position
			self.cmd_position_pub.publish(self.cmd_position_msg)
		else:
			print(__name__, 'send_position_command():', 'Cannot send NoneType value!')

	def send_requests(self, device_id, requests=[]):
		if device_id in self.device_ids and requests:
			self.requests_msg.stamp = rospy.get_rostime()
			self.requests_msg.device_id = device_id
			self.requests_msg.requests = requests
			self.requests_pub.publish(self.requests_msg)

	def heartbeat_request(self):
		requests_list = [PacketID.POSITION, PacketID.VELOCITY, PacketID.CURRENT, PacketID.MODE]
		while not rospy.is_shutdown():	
			if self.device_ids:		
				for device_id in self.device_ids: 					
					self.send_requests(device_id, requests_list)					
					time_sent = rospy.get_time()
					self.wait_for_heatbeat = True
					while rospy.get_time() < (time_sent + self.request_timeout):
						if not self.wait_for_heatbeat:
							print("Received requests from device_Id", device_id, 'in', round(rospy.get_time()-time_sent, 3), 'seconds.')
							break
					if self.wait_for_heatbeat:
						print("#### Missed requests from device_id", device_id, 'in', round(rospy.get_time()-time_sent, 3), 'seconds.')
					self.wait_for_heatbeat = False	
					# Wait for heartbeat frequency, otherwise pass to next interation
					if (rospy.get_time()-time_sent) < (1.0/self.heartbreat_frequency): 
						time.sleep((1.0/self.heartbreat_frequency) - (rospy.get_time()-time_sent))
					else:			
						time.sleep(0.005)
			else:
				break

	'''
	ROS Message Callback Functions
	'''
	def position_callback(self, message):
		if self.wait_for_heatbeat:
			self.wait_for_heatbeat = False
		# if message.device_id in self.device_ids:
		# 	print(__name__, 'position_callback() with Reach5Mini instance', str(self.r5m_instance) + ':', 
		# 		'Message received (device_id, value, timestamp):',	
		# 		message.device_id, message.value, message.stamp.to_sec())

	def velocity_callback(self, message):
		if self.wait_for_heatbeat:
			self.wait_for_heatbeat = False
		# if message.device_id in self.device_ids:
		# 	print(__name__, 'velocity_callback() with Reach5Mini instance', str(self.r5m_instance) + ':', 
		# 		'Message received (device_id, value, timestamp):',	
		# 		message.device_id, message.value, message.stamp.to_sec())

	def current_callback(self, message):
		if self.wait_for_heatbeat:
			self.wait_for_heatbeat = False
		# if message.device_id in self.device_ids:
		# 	print(__name__, 'current_callback() with Reach5Mini instance', str(self.r5m_instance) + ':', 
		# 		'Message received (device_id, value, timestamp):',	
		# 		message.device_id, message.value, message.stamp.to_sec())

	def openloop_callback(self, message):
		if self.wait_for_heatbeat:
			self.wait_for_heatbeat = False
		# if message.device_id in self.device_ids:
		# 	print(__name__, 'openloop_callback() with Reach5Mini instance', str(self.r5m_instance) + ':', 
		# 		'Message received (device_id, value, timestamp):',	
		# 		message.device_id, message.value, message.stamp.to_sec())

	def mode_callback(self, message):
		if self.wait_for_heatbeat:
			self.wait_for_heatbeat = False		
		# if message.device_id in self.device_ids:
		# 	print(__name__, 'mode_callback() with Reach5Mini instance', str(self.r5m_instance) + ':', 
		# 		'Message received (device_id, value, timestamp):',	
		# 		message.device_id, message.value, message.stamp.to_sec())

	def other_callback(self, message):
		if self.wait_for_heatbeat:
			self.wait_for_heatbeat = False		
		if message.device_id in self.device_ids:
			data_array = []
			# Convert incoming float data to int if packet_id does not use float type
			if message.packetid not in FLOAT_PACKETS:
				for data in message.data:
					data_array.append(int(round(data)))	# Making sure that it rounds correctly to the int from the float
			else:
				data_array = message.data
			print(__name__, 'other_callback() with Reach5Mini instance', str(self.r5m_instance) + ':', 
				'Message received (device_id, packet_id, data, timestamp):', 
				message.device_id, message.packet_id, data_array, message.stamp.to_sec())




'''
Test main function
'''
if __name__ == "__main__":
	# Initialise the ROS nodes
    rospy.init_node("r5m_ros_control", anonymous=True)
    myargs = rospy.myargv(argv=sys.argv)	

    # DEFAULTS
    instance = 0

    if myargs and len(myargs) == 2:
	    print('myargs:',end=' ')
	    for arg in myargs:
	        print(arg, end=' ')
	    print('')
	    instance = int(myargs[1])
    else:
	    print('No arguments. Using default settings:', 'instance=', instance)

    # Create Reach5MiniROSController instance (with instance number '0')
    r5m_0 = Reach5MiniROSController(r5m_instance=instance)
    r5m_0.start()

    # Allow callback functions to operate (will continue spin until ROS shutdown or program error)
    rospy.spin()
