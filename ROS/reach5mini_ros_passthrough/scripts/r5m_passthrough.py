#!/usr/bin/env python3

import os, sys, time
import rospy
import serial
import re
import struct
import threading
from blueprintlab_reachsystem_ros_messages.msg import single_float
from blueprintlab_reachsystem_ros_messages.msg import single_int
from blueprintlab_reachsystem_ros_messages.msg import generic
from blueprintlab_reachsystem_ros_messages.msg import request_list
from cobs import cobs
from crcmod import crcmod
from RS1_hardware import PacketID
from packetid import Packets

CRC8_FUNC = crcmod.mkCrcFun(0x14D, initCrc=0xFF, xorOut=0xFF)
FLOAT_PACKETS = Packets().get_float_packet_ids()



'''
SerialConversion class. Functions decodn and encode data for serial transmission.
'''
class SerialConversion():    

    def convert_to_bytes(self, parameter):
        if type(parameter).__name__ == 'float':
            bytes_value = bytearray(struct.pack("f", parameter))
        elif type(parameter).__name__ == 'int':
            bytes_value = bytearray(struct.pack("i", parameter))
        elif type(parameter).__name__ == 'list':
            bytes_value= bytearray()
            for val in parameter:
                bytes_value +=bytearray(struct.pack("f", val))
        else:
            return parameter
        return bytes_value

    # Encode serial packet
    def encode_packet(self, device_id, packet_id, data_in):
        txData = self.convert_to_bytes(data_in)
        datalength = len(txData)
        packet_length = datalength + 4
        txPacket = txData
        txPacket.append(packet_id)
        txPacket.append(device_id)
        txPacket.append(packet_length)
        crcValue=CRC8_FUNC(txPacket)
        txPacket.append(crcValue)
        ## COBS ENCODE
        encoded = cobs.encode(txPacket)
        ## ADD TERMINATOR BYTE
        packet = encoded + b"\x00"
        return packet

    # Decode serial packets
    def parsepacket(self, packet_in):
        if packet_in != b'' and len(packet_in) > 3:
            decoded_pckt = cobs.decode(packet_in)  ## Cobs decode
            if decoded_pckt[-2] != len(decoded_pckt): # check that length is correct. Generally would only happen on first packet or due to bad transmission
                print(__name__, 'parsepacket():', 'decode error!')
            else:
                crcCheck=CRC8_FUNC(decoded_pckt[:-1])
                rxCRC=decoded_pckt[-1]
                if crcCheck == rxCRC:
                    rxDeviceId=decoded_pckt[-3]
                    if rxDeviceId == 0x06:
                        return 0, 0, 0
                    rxPacketId=decoded_pckt[-4]
                    rxData=decoded_pckt[:-4]
                    decoded_pckt = b''
                    rxData = bytearray(rxData) # convert list to bytearray
                    if rxPacketId in FLOAT_PACKETS:
                        data_to_process = rxData
                        rxData = []
                        for i in range(int(len(data_to_process) / 4)):
                            this_float_byte = data_to_process[0:4]
                            data_to_process = data_to_process[4:]
                            rxData.append(struct.unpack("f", this_float_byte)[0])
                    else:
                        out_data = []
                        for b in rxData:
                            out_data.append(b)
                        rxData = out_data
                    return rxDeviceId, rxPacketId, rxData
                else:
                    print(__name__, 'parsepacket():', "CRC Error", [" 0x%02x" % b for b in packet_in])
        return 0, 0, 0




'''
CommConnection class. Write/Read operations for serial data.
'''
class CommConnection():
    serial_device = None
    incomplete_pckt=b''
    send_freq_queue = bytearray([])
    serial_lock = False
    serialconversion = SerialConversion()

    def open(self, port):
        self.serial_device = serial.Serial(port, baudrate=115200, timeout = 3.0)
        if not self.serial_device.is_open:
            raise Exception('Could not connect to port: ' + port)
        
    def _read_serial(self, len=4096):
        if self.serial_device and self.serial_device.is_open:
            bytesToRead = 0
            try:
                bytesToRead = self.serial_device.in_waiting
            except:
                self.close()
                return []
            try:
                if bytesToRead:
                    try:
                        newBytes = self.serial_device.read(bytesToRead)
                        print('got serial data incoming!')
                    except TypeError:
                        return []
                    buff = self.incomplete_pckt + newBytes
                    try:
                        packets = re.split(b'\x00', buff)
                        if buff[-1] != b'0x00':
                            self.incomplete_pckt = packets.pop()
                        return packets
                    except IndexError:
                        return []
                return []
            except serial.SerialException:
                self.close()
                print(__name__, "_read_serial():", "Serial device error. Is the device connected and working?")
                return []
            except:
                self.connected = False
                return []
        else:
            print(__name__, "_read_serial():", "Serial device is not open")

    def _send_serial(self, device_id, packet_id, data_in=[], append_only=False):
        if self.serial_device and self.serial_device.is_open:
            ts = time.time()
            try:
                data = data_in
                if data_in is not None and data_in != []:
                    if device_id is not None:
                    	data = self.serialconversion.encode_packet(device_id, packet_id, data_in)
                    self.send_freq_queue += data
                if append_only:
                    return 1                
                s = self.serial_device.write(self.send_freq_queue)
                self.send_freq_queue = bytearray([])
                return s
            except serial.SerialTimeoutException:
                print(__name__, "_send_serial():", "Serial write timeout (timeout:", str(self.serial_device.timeout), str(self.serial_device.write_timeout), "s, recorded time:", time.time()-ts, "s). Is the device connected and working?")
                return 0
            except serial.SerialException:
                self.send_freq_queue = bytearray([])
                self.serial_device.close()
                print(__name__, "_send_serial():", "Serial device error. Is the device connected and working?")
                return -1
            except Exception as e:
                print(__name__, "_send_serial():", "Send Error:", e)
                self.send_freq_queue = bytearray([])
                return -1
        else:
            self.serial_device.close()
            print(__name__, "_send_serial():", "Serial device is not open")
            return -1




'''
Reach5Mini_Passthrough class. ROS to Serial operations and vice versa. 
'''
class Reach5Mini_Passthrough():
	instances = []

	def __init__(self, r5m_instance=0, device_ids=[1,2,3,4,5], **kwargs):
		self.r5m_instance = r5m_instance
		self.device_ids = 	device_ids

		for r5m_inst in Reach5Mini_Passthrough.instances:
			if r5m_inst.r5m_instance == self.r5m_instance:
				raise Exception('Cannot use the same r5m_instance number as other Reach5Mini_Passthrough class instances!')
		Reach5Mini_Passthrough.instances.append(self)

		self.commconnection = CommConnection()
		self.serialconversion = SerialConversion()

		# Initialise ROS publishers and subscribers
		self.cmd_velocity_sub =     rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/cmd_velocity', single_float, self.cmd_velocity_callback)  
		self.cmd_position_sub =     rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/cmd_position', single_float, self.cmd_position_callback)    
		self.requests_sub =     	rospy.Subscriber('r5m_' + str(self.r5m_instance) + '/requests', request_list, self.requests_callback)  
		self.position_pub  = 	    rospy.Publisher('r5m_' + str(self.r5m_instance) + '/position', single_float, queue_size=10)
		self.velocity_pub = 	    rospy.Publisher('r5m_' + str(self.r5m_instance) + '/velocity', single_float, queue_size=10)
		self.current_pub = 	    	rospy.Publisher('r5m_' + str(self.r5m_instance) + '/current', single_float, queue_size=10)
		self.openloop_pub = 	    rospy.Publisher('r5m_' + str(self.r5m_instance) + '/openloop', single_float, queue_size=10)
		self.mode_pub = 	    	rospy.Publisher('r5m_' + str(self.r5m_instance) + '/mode', single_int, queue_size=10)
		self.other_pub =            rospy.Publisher('r5m_' + str(self.r5m_instance) + '/other', generic, queue_size=10)

		# Initialise ROS messages
		self.position_message = 	single_float()
		self.velocity_message =		single_float()
		self.current_message = 	  	single_float()
		self.openloop_message =   	single_float()
		self.mode_message = 	    single_int()
		self.other_message =        generic()        

	'''
	Main Script - used to create necessary threads
	'''
	def start(self):        
		if self.commconnection.serial_device is None:
			self.commconnection.open('/dev/ttyUSB0')
		if self.commconnection.serial_device.is_open:
			self.thread_main_loop = threading.Thread(target=self.read_serial_and_publish_data_loop)
			self.thread_main_loop.setDaemon(True)
			self.thread_main_loop.start()
			print(__name__, 'start():', 'Callback spin started!')
	
	'''
	Incoming data passthrough Serial to ROS functions	
	'''		
	def read_serial_and_publish_data_loop(self):
		print(__name__, 'read_serial_and_publish_data_loop():', 'Started main serial read and publish thread!')
		while not rospy.is_shutdown():
			time.sleep(0.001)
			while self.commconnection.serial_lock:
				time.sleep(0.001)
			self.commconnection.serial_lock = True
			packets = self.commconnection._read_serial()
			self.commconnection.serial_lock = False
			if packets:
				for packet in packets:
					device_id, packet_id, data = self.serialconversion.parsepacket(packet)
					if device_id == 0:
						continue
					if packet_id == PacketID.POSITION:
						self.position_message.stamp = rospy.get_rostime()
						self.position_message.device_id = device_id
						self.position_message.value = data[0]
						self.position_pub.publish(self.position_message)
					elif packet_id == PacketID.VELOCITY:
						self.velocity_message.stamp = rospy.get_rostime()
						self.velocity_message.device_id = device_id
						self.velocity_message.value = data[0]
						self.velocity_pub.publish(self.velocity_message)
					elif packet_id == PacketID.CURRENT:
						self.current_message.stamp = rospy.get_rostime()
						self.current_message.device_id = device_id
						self.current_message.value = data[0]
						self.current_pub.publish(self.current_message)
					elif packet_id == PacketID.OPENLOOP:
						self.openloop_message.stamp = rospy.get_rostime()
						self.openloop_message.device_id = device_id
						self.openloop_message.value = data[0]
						self.openloop_pub.publish(self.openloop_message)                    
					elif packet_id == PacketID.MODE:
						self.mode_message.stamp = rospy.get_rostime()
						self.mode_message.device_id = device_id
						self.mode_message.value = data[0]
						self.mode_pub.publish(self.mode_message)
					else:
						self.other_message.stamp = rospy.get_rostime()
						self.other_message.device_id = device_id
						self.other_message.packet_id = packet_id
						data_float = []                    
						for data_int in data:
							data_float.append(float(data_int))
						self.other_message.data = data_float
						self.other_pub.publish(self.other_message)

	'''
	ROS Message Callback Functions
	'''        
	def cmd_velocity_callback(self, message):
		if self.commconnection.serial_device:
			print(__name__, 'cmd_velocity_callback():', 'device_id, value, timestamp:', 
				message.device_id, message.value, message.stamp.to_sec())
			while self.commconnection.serial_lock:
				time.sleep(0.001)
			self.commconnection.serial_lock = True            
			self.commconnection._send_serial(message.device_id, PacketID.VELOCITY, message.value)
			self.commconnection.serial_lock = False

	def cmd_position_callback(self, message):
		if self.commconnection.serial_device:
			print(__name__, 'cmd_position_callback():', 'device_id, value, timestamp:', 
				message.device_id, message.value, message.stamp.to_sec())
			while self.commconnection.serial_lock:
				time.sleep(0.001)
			self.commconnection.serial_lock = True            
			self.commconnection._send_serial(message.device_id, PacketID.POSITION, message.value)
			self.commconnection.serial_lock = False 

	def requests_callback(self, message):
		if self.commconnection.serial_device:
			requests_to_send = bytearray([])
			for rq in message.requests:
				requests_to_send.append(rq)
			print(__name__, 'requests_callback():', 'device_id, value, timestamp:', 
				message.device_id, requests_to_send, message.stamp.to_sec())
			while self.commconnection.serial_lock:
				time.sleep(0.001)
			self.commconnection.serial_lock = True            
			self.commconnection._send_serial(message.device_id, PacketID.REQUEST_PACKET, requests_to_send)
			self.commconnection.serial_lock = False    




'''
Main function - requires arguments: R5M instance number, serial port (eg. 0, /dev/ttyUBS0)
Example run command: rosrun reach5mini_ros_passthrough r5m_passthrough.py 0 /dev/ttyUSB0
'''
if __name__ == "__main__":
	# Initialise the ROS nodes
	rospy.init_node("r5m_passthrough", anonymous=True)
	myargs = rospy.myargv(argv=sys.argv)	

	# DEFAULTS
	instance = 0
	port = '/dev/ttyUSB0'

	if myargs and len(myargs) == 3:
		print('myargs:',end=' ')
		for arg in myargs:
			print(arg, end=' ')
		print('')
		instance = int(myargs[1])
		port = myargs[2]
	else:
		print('No arguments. Using default settings:', 'instance=', instance, 'port=', port)
	# Create Reach5Mini_Passthrough instance (with instance number '0') and connect to serial device
	
	r5m_0_passthrough = Reach5Mini_Passthrough(instance)
	try:
		r5m_0_passthrough.commconnection.open(port)
		r5m_0_passthrough.start()
		rospy.spin()
	except:
		print('Could not connect to', port, 'killing node!')
		exit(1)
	

	# Allow callback functions to operate (will continue spin until ROS shutdown or program error)
	  
