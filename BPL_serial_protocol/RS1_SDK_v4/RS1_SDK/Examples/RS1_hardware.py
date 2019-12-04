"""RS1 Hardware specific definitions"""

import inspect


class PacketID():
	"""PacketID Class: 8-bit definitions of the commands/utilities packet ids 
	that can be sent/received from the product devices
	"""
	MODE = 0x01
	VELOCITY = 0x02
	POSITION = 0x03
	OPENLOOP = 0x04
	CURRENT = 0x05

	SUPPLYVOLTAGE = 0x90
	TEMPERATURE = 0x66
	REQUEST_PACKET = 0x60
	SERIAL_NUMBER = 0x61 
	MODEL_NUMBER = 0x62 
	VERSION = 0x63 
	DEVICE_ID = 0x64
	INTERNAL_HUMIDITY = 0x65
	INTERNAL_TEMPERATURE = 0x66
	DEVICE_TYPE = 0x67
	HARDWARE_STATUS = 0x68
	RUN_TIME = 0x69
	STATE_ESTIMATOR_STATUS = 0x71

	COMS_PROTOCOL = 0x80
	HEARTBEAT_FREQUENCY_SET = 0x92
	HEARTBEAT_SET = 0x91

	SAVE = 0x50
	LOAD = 0x51
	SET_DEFAULTS = 0x52
	FORMAT = 0x53

	CHANGE_PAGE = 0x54

	CURRENT_LIMIT = 0x12
	VELOCITY_LIMIT = 0x11
	POSITION_LIMIT = 0x10
	POSITION_GAIN = 0x13
	VELOCITY_GAIN = 0x14
	CURRENT_GAIN = 0x15

	POSITION_PARAMETERS = 0x20       
	VELOCITY_PARAMETERS = 0x21      
	CURRENT_PARAMETERS = 0x22        
	INPUT_VOLTAGE_PARAMETERS = 0x23 

	MOTOR_PARAMETERS = 0x30          

	MAX_ACCELERATION = 0x40
	CURRENT_HOLD_THRESHOLD = 0x41
	COMPLIANCE_GAIN = 0x42
	COMPLIANCE_PARAMETERS = 0x44

	# 3 bytes for each version packet
	# [major, submaj, minor]
	ELECTRICAL_VERSION = 0x6A
	MECHANICAL_VERSION = 0x6B
	SOFTWARE_VERSION = 0x6C

	BOOTLOADER_STM = 0xFE
	BOOTLOADER = 0xFF

	KM_CONFIGURATION = 0xA0  
	KM_END_POS = 0xA1  
	KM_END_VEL = 0xA2  

	KM_BOX_OBSTACLE_00 = 0xA3  
	KM_BOX_OBSTACLE_01 = 0xA4
	KM_BOX_OBSTACLE_02 = 0xA5
	KM_BOX_OBSTACLE_03 = 0xA6
	KM_BOX_OBSTACLE_04 = 0xA7
	KM_BOX_OBSTACLE_05 = 0xA8
	KM_CYLINDER_OBSTACLE_00 = 0xA9  
	KM_CYLINDER_OBSTACLE_01 = 0xAA
	KM_CYLINDER_OBSTACLE_02 = 0xAB
	KM_CYLINDER_OBSTACLE_03 = 0xAC
	KM_CYLINDER_OBSTACLE_04 = 0xAD
	KM_CYLINDER_OBSTACLE_05 = 0xAE

	KM_FLOAT_PARAMETERS = 0xB0  
	
	KM_JOINT_STATE = 0xB2
	KM_JOINT_STATE_REQUEST = 0xB3

	KM_DH_PARAMETERS_0 = 0xB8
	KM_DH_PARAMETERS_1 = 0xB9
	KM_DH_PARAMETERS_2 = 0xBA
	KM_DH_PARAMETERS_3 = 0xBB
	KM_DH_PARAMETERS_4 = 0xBC
	KM_DH_PARAMETERS_5 = 0xBD
	KM_DH_PARAMETERS_6 = 0xBE
	KM_DH_PARAMETERS_7 = 0xBF

	KM_POS_LIMIT_TRANSLATE  = 0xC0
	KM_VEL_LIMIT_TRANSLATE  = 0xC1
	KM_POS_LIMIT_YAW        = 0xC2
	KM_POS_LIMIT_PITCH      = 0xC3
	KM_POS_LIMIT_ROLL       = 0xC4
	KM_VEL_LIMIT_ROTATE     = 0xC5
	KM_POS_GAINS_TRANSLATE  = 0xC6
	KM_VEL_GAINS_TRANSLATE  = 0xC7
	KM_POS_GAINS_ROTATE     = 0xC8
	KM_VEL_GAINS_ROTATE     = 0xC9

	KM_JOINT_POS_0 = 0xD0
	KM_JOINT_POS_1 = 0xD1
	KM_JOINT_POS_2 = 0xD2
	KM_JOINT_POS_3 = 0xD3
	KM_JOINT_POS_4 = 0xD4
	KM_JOINT_POS_5 = 0xD5
	KM_JOINT_POS_6 = 0xD6
	KM_JOINT_POS_7 = 0xD7

	KM_COLLISION_FLAG = 0xAF
	KM_COLLISION_COORDS = 0xB1


	ENCODER = 0xE0
	ENCODER_PARAMETERS = 0xE1

	TEST_PACKET = 0xE2

	MODE_SETTINGS = 0x43    

	RESET = 0xFD

'''
FLOAT_PACKETS: 	List of all of the PacketIDs that sends/receives 
				floating point (32-bit) data rather than integers
'''
FLOAT_PACKETS = [PacketID.VELOCITY, PacketID.POSITION, PacketID.OPENLOOP, PacketID.CURRENT,
				 PacketID.SUPPLYVOLTAGE, PacketID.TEMPERATURE,
				 PacketID.MODEL_NUMBER,
				 PacketID.SERIAL_NUMBER, PacketID.VERSION,
				 PacketID.RUN_TIME,
				 PacketID.INTERNAL_HUMIDITY, PacketID.INTERNAL_HUMIDITY,
				 PacketID.MAX_ACCELERATION,
				 PacketID.POSITION_PARAMETERS, PacketID.VELOCITY_PARAMETERS, PacketID.CURRENT_PARAMETERS,
				 PacketID.INPUT_VOLTAGE_PARAMETERS,
				 PacketID.MOTOR_PARAMETERS, PacketID.MOTOR_PARAMETERS, PacketID.CURRENT_HOLD_THRESHOLD,
				 PacketID.CURRENT_HOLD_THRESHOLD,
				 PacketID.COMPLIANCE_GAIN,
				 PacketID.CURRENT_LIMIT, PacketID.VELOCITY_LIMIT, PacketID.POSITION_LIMIT,
				 PacketID.POSITION_GAIN, PacketID.VELOCITY_GAIN, PacketID.CURRENT_GAIN,
				 PacketID.KM_END_POS, PacketID.KM_END_VEL,
				 PacketID.KM_BOX_OBSTACLE_00, PacketID.KM_BOX_OBSTACLE_01, PacketID.KM_BOX_OBSTACLE_02,
				 PacketID.KM_BOX_OBSTACLE_03, PacketID.KM_BOX_OBSTACLE_04, PacketID.KM_BOX_OBSTACLE_05,
				 PacketID.KM_CYLINDER_OBSTACLE_00, PacketID.KM_CYLINDER_OBSTACLE_01, PacketID.KM_CYLINDER_OBSTACLE_02,
				 PacketID.KM_CYLINDER_OBSTACLE_03, PacketID.KM_CYLINDER_OBSTACLE_04, PacketID.KM_CYLINDER_OBSTACLE_05,
				 PacketID.KM_DH_PARAMETERS_0, PacketID.KM_DH_PARAMETERS_1,
				 PacketID.KM_DH_PARAMETERS_2, PacketID.KM_DH_PARAMETERS_3,
				 PacketID.KM_DH_PARAMETERS_4, PacketID.KM_DH_PARAMETERS_5,
				 PacketID.KM_DH_PARAMETERS_6, PacketID.KM_DH_PARAMETERS_7,
				 PacketID.KM_FLOAT_PARAMETERS,
				 PacketID.KM_POS_LIMIT_TRANSLATE, PacketID.KM_VEL_LIMIT_TRANSLATE,
				 PacketID.KM_POS_LIMIT_YAW, PacketID.KM_POS_LIMIT_PITCH, PacketID.KM_POS_LIMIT_ROLL,
				 PacketID.KM_VEL_LIMIT_ROTATE,
				 PacketID.KM_POS_GAINS_TRANSLATE, PacketID.KM_VEL_GAINS_TRANSLATE,
				 PacketID.KM_POS_GAINS_ROTATE, PacketID.KM_VEL_GAINS_ROTATE, PacketID.COMPLIANCE_PARAMETERS]


class Mode():
	"""Mode Class: 8-bit definitions of the device Modes that can be sent/received from the product devices"""
	STANDBY = 0x00
	DISABLE = 0x01
	VELOCITY = 0x03
	POSITION = 0x02
	OPENLOOP = 0x05
	FACTORY = 0x08
	CURRENT = 0x04
	CALIBRATE = 0x07
	INIT = 0x06
	COMPLIANT = 0x09
	GRIP = 0x0A


def get_name_of_mode(mode):
	"""Get MODE name from packet_id (see Mode class) 
		Eg. If mode = 5 (0x05 in hexidecimal), function will return 'OPENLOOP'
	"""
	packet_list = get_list_of_modes()
	for member in packet_list:
		if member[1] == mode:
			return member[0]
	return 'UNKNOWN'

class Gains():
	KP = 0
	KI = 0
	KD = 0
	KF = 0


class HardwareStatus():
	ERROR_NOT_DETECTED = 0
	ENCODER_POSITION_ERROR = 1
	MOTOR_DRIVER_FAULT = 2
	COMS_CRC_ERROR = 3
	COMS_SERIAL_ERROR = 4
	HARDWARE_OVER_TEMPERATURE = 5
	HARDWARE_OVER_HUMIDITY = 6
	FLASH_FAILED_READ = 7


class KinematicsConfigFrames():
	NO_CHANGE = 0
	WORLD = 1
	BASE = 2
	END = 3


class KinematicsFrames():
	NO_CHANGE = 0
	WORLD = 1
	BASE = 2
	END = 3


class KinematicsConfigOrientation():
	NO_CHANGE = 0
	UPRIGHT = 1
	INVERTED = 2


class KinematicsConfigEnable():
	NO_CHANGE = 0
	DISABLE = 1
	ENABLE = 2


class KinematicsConfigObstacleEnable():
	NO_CHANGE = 0
	DISABLE = 1
	ENABLE = 2


class DeviceType:
	ROTATE = 0
	LINEAR = 1


class NCValue:
	NO_CHANGE = 0
	DISABLE = 1
	ENABLE = 2


class test():

	check=0

	def testFunction(self):
		self.check=self.check+1
		print (self.check)


def get_list_of_packets():
	"""Returns alphabetised list of tuples containing all packet ids from PacketID() class
		e.g.: [('BOOTLOADER', 255), ('COMPLIANCE_GAIN', 66), ('COMS_PROTOCOL', 128), ('CURRENT', 5), ...]
	"""
	attrlist = inspect.getmembers(PacketID)
	list_of_packets = []
	for member in attrlist:
		if not member[0].startswith('_'):
			list_of_packets.append(member)
	return list_of_packets


def get_list_of_modes():
	"""Returns list of Modes (see Mode class)"""
	attrlist = inspect.getmembers(Mode)
	list_of_packets = []
	for member in attrlist:
		if not member[0].startswith('_'):
			list_of_packets.append(member)
	return list_of_packets


def get_name_of_packet_id(packet_id):
	"""Get PacketID name from packet_id (see PacketID class) 
		Eg. If packet_id = 2 (0x02 in hexidecimal), function will return 'VELOCITY'
	"""
	packet_list = get_list_of_packets()
	for member in packet_list:
		if member[1] == packet_id:
			return member[0]
	return 'UNKNOWN'


def get_hardware_status_string(status):
	"""Returns string representing the name of the hardware status int provided."""
	attrlist = inspect.getmembers(HardwareStatus)
	for member in attrlist:
		if not member[0].startswith('_'):
			if member[1] == status:
				return member[0]
	return 'UNKNOWN_ERROR'
