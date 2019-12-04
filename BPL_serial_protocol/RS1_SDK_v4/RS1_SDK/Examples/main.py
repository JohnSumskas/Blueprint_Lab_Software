'''THE BLUEPRINT LABORATORY

Reach Control SDK
Basic Implementation Guide
This software and all included documents are confidential and not for re-distrbution.

********************
v4.0
04/Dec/2019
By Shaun Barlow and Jean-Luc Stevens
********************

The following program contains basic examples on how to control
and read data from an RS1 dual function grabber.

***IMPORTANT***
The user must set the name of the COM_PORT variable below.
To find the correct comport name, use the Reach5Mini.usb_list() function.

INSTRUCTIONS
This example code can be run by executing the file: "python path_to_main/main.py"
or via an interactive python terminal:
# python
>> from RS_integration_example import Reach5Mini
>> rs = Reach5Mini()
>> rs.connect()
>> rs.enable_all()
>> rs.set_position(1, 10)

An example sequence can be seen in the last section of this document
and may be edited to test the various functions found in Reach5Mini class.
Further specifics of use are documented in that section.

Functions requiring a "deviceID" argument (e.g. get_position(deviceID)):
Reach5Mini.deviceIDs[0] refers to the grabber,
Reach5Mini.deviceIDs[1] refers to the rotate joint used to rotate the grabber,
Reach5Mini.deviceIDs[4] refers to the main I/O device on the Reach5Mini.

NOTES ON RECEIVING INFO FROM RS1
The RS1 device may be set up with a heartbeat (see Comms Manual for further details on this).
When heartbeat is used, it is redundant to manually request data that is already included in the heartbeat.
This example code assumes that no hardware heartbeat is set and always sends a manual request for data.
When hardware heartbeat is used, the lines:
        self.global_serial.send_request_packet(...
        time.sleep(0.1) # give the serial protocol 0.1 seconds to receive the message
may be removed from all functions below.
If your software has scope for a python-side heartbeat/loop/clock/systick function, then run the
    self.global_serial.update_motor(1)
command in the python-side heartbeat function and remove it from the get_...() functions below.

'''

'''USER SETTINGS'''
COM_PORT = 'COM30'      # Name of the comport connected to RS (use COM port name if on Windows)
# COM_PORT = '/dev/ttyUSB0'     # Name of the comport connected to RS (use ports in /dev/ folder for name Linux)
BAUD = 115200   # Should not need changing from 115200

import time
from serial_device import SerialDevice
from RS1_hardware import PacketID, Mode
from bplprotocol import BplSerial
import serial.tools.list_ports



class Reach5Mini():
    """Reach5Mini Class: Example class used for interfacing with the Reach5Mini attached on the serial port (COM_PORT)
        Has example functionality for sending commands (eg. position, velocity, mode),
        and requesting data (eg. position, current, mode, serial number).
    """
    # 5 device IDs to allow for the 5 axis config.
    deviceIDs = [0x01, 0x02, 0x03, 0x04, 0x05]
    # create an instance of SerialDevice()
    serial_dev = SerialDevice()
    # that instance is passed as argument to BplSerial
    global_serial = BplSerial(serial_dev, len(deviceIDs))
    # If more than one serial device is required (i.e. for multiple grabbers),
    # create additional instances of SerialDevice() and BplSerial()
    # EG: serial_dev_2 = SerialDevice()
    # global_serial_2 = BplSerial(serial_dev_2)

    global_serial.deviceIDs = deviceIDs

    def connect(self):
        self.global_serial.connect(COM_PORT, BAUD)

    def usb_list(self):
        '''
        Prints a list of available serial ports.
        This is a helper function and is non-essential.
        :return: None
        '''
        comports = serial.tools.list_ports.comports()
        print('***\nCOM PORT LIST:')
        for comport in comports:
            print(comport, comport.device)
        print('***\n')

    def enable_all(self):
        '''
        Axes must be enabled before they will respond to further commands.
        :return: None
        '''
        for devid in self.deviceIDs:
            self.global_serial.send_mode(devid, Mode.STANDBY)
        print('Reach5Mini enable_all()')

    def disable_all(self):
        '''
        Disable all axes
        :return: None
        '''
        for devid in self.deviceIDs:
            self.global_serial.send_mode(devid, Mode.DISABLE)
        print('Reach5Mini disable_all()')

    def set_position(self, device_id, position):
        """Set device position: in radians for rotate, or millimetre for grabber"""
        self.global_serial.send_position(device_id, position)
        print('Reach5Mini sent position for device', device_id, ':', position)
    
    def set_velocity(self, device_id, velocity):
        """Set device velocity: in radians per second for rotate, or millimetres per second for grabber"""
        self.global_serial.send_velocity(device_id, velocity)
        print('Reach5Mini sent velocity for device', device_id, ':', velocity)

    def set_openloop(self, device_id, openloop):
        """Set device openloop: in radians per second for rotate, or millimetres per second for grabber - 
            THIS CAN BE DANGEROUS, NOT RECOMMENDED
        """
        self.global_serial.send_openloop(device_id, openloop)
        print('Reach5Mini sent openloop for device', device_id, ':', openloop) 
    
    def set_current(self, device_id, current):
        """Set device current: in mA - THIS CAN BE DANGEROUS, NOT RECOMMENDED"""
        self.global_serial.send_current(device_id, current)
        print('Reach5Mini sent current for device', device_id, ':', current) 
    
    def set_mode(self, device_id, mode):
        """Set device mode: refer to MODE list in RS1_hardware.py for details"""
        self.global_serial.send_mode(device_id, mode)
        print('Reach5Mini sent mode for device', device_id, ':', mode) 

    def get_voltage(self):
        '''
        See NOTES ON RECEIVING INFO FROM RS1 at top of document
        for optimisation of this and subsequent get_... functions
        :return: float
        '''
        self.global_serial.send_request_packet(self.deviceIDs[4], PacketID.SUPPLYVOLTAGE)
        time.sleep(0.1) # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(self.deviceIDs[4])
        voltage = this_motor[PacketID.SUPPLYVOLTAGE]
        print('voltage:', str(voltage))
        return voltage

    def get_mode(self, deviceID):
        self.global_serial.send_request_packet(deviceID, PacketID.MODE)
        time.sleep(0.1)     # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(deviceID)
        mode = this_motor[PacketID.MODE]
        print('mode:', str(mode))
        return mode

    def get_current(self, deviceID):
        self.global_serial.send_request_packet(deviceID, PacketID.CURRENT)
        time.sleep(0.1)     # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(deviceID)
        current = this_motor[PacketID.CURRENT]
        print('current:', str(current))
        return current

    def get_velocity(self, deviceID):
        self.global_serial.send_request_packet(deviceID, PacketID.VELOCITY)
        time.sleep(0.1)     # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(deviceID)
        velocity = this_motor[PacketID.VELOCITY]
        print('velocity:', str(velocity))
        return velocity

    def get_position(self, deviceID):
        self.global_serial.send_request_packet(deviceID, PacketID.POSITION)
        time.sleep(0.1)     # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(deviceID)
        position = this_motor[PacketID.POSITION]
        print('position:', str(position))
        return position

    def get_model_number(self):
        self.global_serial.send_request_packet(self.deviceIDs[4], PacketID.MODEL_NUMBER)
        time.sleep(0.1)     # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(self.deviceIDs[4])
        model_number = this_motor[PacketID.MODEL_NUMBER]
        print('model number:', str(model_number))
        return model_number

    def get_serial_number(self):
        self.global_serial.send_request_packet(self.deviceIDs[4], PacketID.SERIAL_NUMBER)
        time.sleep(0.1)     # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(self.deviceIDs[4])
        serial_number = this_motor[PacketID.SERIAL_NUMBER]
        print('serial number:', str(serial_number))
        return serial_number

    def get_firmware_version(self):
        self.global_serial.send_request_packet(self.deviceIDs[4], PacketID.VERSION)
        time.sleep(0.1)     # give the serial protocol 0.1 seconds to receive the message
        self.global_serial.updateMotor(1)
        this_motor = self.global_serial.get_motor_by_device_id(self.deviceIDs[4])
        version = this_motor[PacketID.VERSION]
        print('firmware version:', str(version))
        return version


if __name__ == '__main__':
    '''MAIN function/
    The following sequence provides an example of how to use the Reach5Mini class.
    It may be edited to allow for experimentation with the unit.
    '''
    # Setup Reach5Mini and connect to serial port
    rs = Reach5Mini()           # create an instance of the Reach5Mini class
    rs.usb_list()               # list all usb ports. Can be handy if you're not sure which port the RS1 is connected to.
    rs.connect()                # connect to the serial port listed in the COM_PORT var above
    rs.enable_all()             # Make sure all devices are ready to do stuff
    grabber_device_id = rs.deviceIDs[0]         # Device ID of grabber
    base_device_id = rs.deviceIDs[4]            # Device ID of main I/O device on the Reach5Mini (device_id = 5)
    rs.get_mode(base_device_id) 

    # Get model number, serial number, firmware version, and voltage of base device - all other devices have the same details  
    rs.get_model_number()
    rs.get_serial_number()
    rs.get_firmware_version()
    rs.get_voltage()
    time.sleep(0.1)

    # Send control commands
    rs.set_velocity(grabber_device_id, -1)      # send velocity (in mm/s) to grabber device
    time.sleep(2)       # give it 2 seconds to allow device to move. This is for demonstration purposes only. Not essential.
    rs.set_velocity(grabber_device_id, 1)       # send a different velocity
    time.sleep(2)       # give it 2 seconds to allow device to move. This is for demonstration purposes only. Not essential.
    rs.set_velocity(grabber_device_id, 0)       # send a different velocity
    time.sleep(2)       # give it 2 seconds to allow device to move. This is for demonstration purposes only. Not essential.

    # Read position, velocity, and current from the grabber
    rs.get_position(grabber_device_id)
    rs.get_velocity(grabber_device_id)
    rs.get_current(grabber_device_id)            