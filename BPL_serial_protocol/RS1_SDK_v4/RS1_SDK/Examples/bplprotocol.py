"""Blueprint Lab protocol file. 
    Functions for sending and receiving packets.
"""

from RS1_hardware import PacketID, test, Mode, Gains

def test_func(obj, value):
    print('bplprotocol.test_func called with value:', value)



class BplSerial():
    """BplSerial class. A series of functions to send and receive data from the serial device. 
        Stores data from the devices of the product in the motor_data dictionary.
    """

    """motor_data: Device data storage mechanism as a dictionary.
        Access correct device data given a device_id via: motor = get_motor_by_device_id(device_id).
        To get or set device id from the motor, call motor['deviceID'].
        To get or set data for a specific packet_id from the motor (see RS1_hardware.py/PacketID class), 
            call motor[packet_id] or motor[PacketID.<packet>] (eg. motor[PacketID.MODE] to get current device MODE)
    """
    motor_data = {}

    deviceIDs = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    connection_status = ''
    comport_status = 'closed'

    def __init__(self, serial_dev, num_motors=6):
        self.serial_device = serial_dev
        self._init_motors(num_motors)

    def _init_motors(self, num_motors=6):
        alphalist = ['a', 'b', 'c', 'd', 'e', 'f']
        empty_motor = self._init_motor_model()
        for num in range(0, num_motors):
            motor_name = alphalist[num]
            self.motor_data[motor_name] = empty_motor.copy()
            self.motor_data[motor_name]['deviceID'] = self.deviceIDs[num]

    def _init_motor_model(self):
        empty_motor = {}
        for a in dir(PacketID):
            if not a.startswith('__'):
                pid = getattr(PacketID,a)
                if a.endswith('LIMIT'):
                    empty_motor[pid] = {'max': 0, 'min': 0}
                elif a.endswith('GAIN'):
                    empty_motor[pid] = {'KP': 0, 'KI': 0, 'KD': 0, 'KF': 0}
                else:
                    empty_motor[pid] = 0
        return empty_motor

    def connect(self, port, baud):
        if (port == 'None'):
            self.serial_device.close()
        else:
            self.serial_device.baud = int(baud)
            self.serial_device.port = port
            self.serial_device.comsType = "SERIAL"
            print("Connecting")
            self.comport_status = 'connecting'
            self.serial_device.open()
            if (self.serial_device.serialPort.is_open):
                self.comport_status = 'open'
            else:
                self.comport_status = 'closed'
            self.serial_device.txDeviceId = 0x01
            self.serial_device.rxPacketId = 0x00
            self.serial_device.rxDeviceId = 0x00

    def send_velocity(self, DeviceID, slider_val):
        txDeviceId = DeviceID
        txData = float(slider_val)
        txPacketId = PacketID.VELOCITY
        self.serial_device.sendpacket(txDeviceId, txPacketId, float(txData))

    def send_openloop(self, DeviceID, slider_val):
        txDeviceId = DeviceID
        txData = slider_val/100
        txPacketId=PacketID.OPENLOOP
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_position(self,DeviceID, slider_val):
        txDeviceId = DeviceID
        txData = slider_val*6.2831853/360.0
        txPacketId=PacketID.POSITION
        self.serial_device.sendpacket(txDeviceId, txPacketId, float(txData))

    def send_current(self, DeviceID, slider_val):
        txDeviceId = DeviceID
        txData = slider_val
        txPacketId=PacketID.CURRENT
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_mode(self, DeviceID, mode):
        if mode in [Mode.STANDBY, Mode.DISABLE, Mode.VELOCITY, Mode.POSITION, Mode.OPENLOOP, Mode.FACTORY, Mode.CURRENT,
                    Mode.CALIBRATE, Mode.INIT, Mode.COMPLIANT, Mode.GRIP]:
            txDeviceId = DeviceID
            txPacketId = PacketID.MODE
            self.serial_device.sendpacket(txDeviceId, txPacketId, mode)

    ### CONFIG FUNCTIONS ###
    def send_factoryMode(self, DeviceID, checkboxValue):
        if checkboxValue == False:
            txData = Mode.DISABLE
        else:
            txData = Mode.FACTORY
        txDeviceId= DeviceID
        txPacketId=PacketID.MODE
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_current_limit(self, DeviceID, max, min):
        txDeviceId = DeviceID
        txData = ([max, min])
        txPacketId = PacketID.CURRENT_LIMIT
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_velocity_limit(self,DeviceID, max, min):
        txDeviceId=DeviceID
        txData = ([max*6.2831853/360.0, min*6.2831853/360.0])
        txPacketId=PacketID.VELOCITY_LIMIT
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_position_limit(self,DeviceID, max, min):
        txDeviceId=DeviceID
        txData = ([max*6.2831853/360.0, min*6.2831853/360.0])
        txPacketId=PacketID.POSITION_LIMIT
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_position_gains(self,DeviceID,gain):
        self.send_gains(DeviceID, gain, PacketID.POSITION_GAIN)

    def send_velocity_gains(self,DeviceID,gain):
        self.send_gains(DeviceID, gain, PacketID.VELOCITY_GAIN)

    def send_current_gains(self, DeviceID, gain):
        self.send_gains(DeviceID, gain, PacketID.CURRENT_GAIN)

    def send_gains(self,DeviceID,gain,packetId):
        txDeviceId=DeviceID
        txData = ([gain.kp,gain.ki,gain.kd,gain.kf])
        txPacketId=packetId
        print(txPacketId)
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_heartrate_set(self, DeviceID, value):
        txData = bytearray([int(value)])
        print('HEARTRATE', txData, int(value))
        txDeviceId= DeviceID
        txPacketId= PacketID.HEARTBEAT_FREQUENCY_SET
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_manual_packet(self, DeviceID, PacketID, d1, d2, d3, d4):
        txDeviceId=DeviceID
        txData = (bytearray([d1, d2, d3, d4]))
        txPacketId = PacketID
        print(txPacketId)
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_request_packet(self, DeviceID, packetIDRequest):
        # print('BPLprotocol: send request packet. DevID:', DeviceID, 'Packet IDs requested:', packetIDRequest)
        txData = packetIDRequest
        txDeviceId = DeviceID
        txPacketId = PacketID.REQUEST_PACKET
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_model_number(self, DeviceID, value):
        txData = value
        txDeviceId = DeviceID
        txPacketId = PacketID.MODEL_NUMBER
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_serial_number(self, DeviceID, value):
        txData = value
        txDeviceId = DeviceID
        txPacketId = PacketID.SERIAL_NUMBER
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_bootloader(self, DeviceID):
        txDeviceId = DeviceID
        txData = 0
        txPacketId = PacketID.BOOTLOADER
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_save_configuration(self, DeviceID):
        txData = 0
        txDeviceId= DeviceID
        txPacketId=PacketID.SAVE
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)

    def send_load_configuration(self, DeviceID):
        txData = 0
        txDeviceId= DeviceID
        txPacketId=PacketID.LOAD
        self.serial_device.sendpacket(txDeviceId, txPacketId, txData)
    ### END CONFIG FUNCTIONS ###


    def get_motor_by_device_id(self, deviceID):
        """Returns correct data storage dictionary (from motor_data dictionary) from deviceID"""
        for key, value in self.motor_data.items():
            if value['deviceID'] == deviceID:
                # print('GMBD found motor', key, 'deviceID', value['deviceID'])
                return value
        print('ERROR: No motor with deviceID:', deviceID)
        return None


    def updateMotor(self, dt):
        """updateMotor: Receives serial data and pushes data to its correct slot (in motor_data dictionary) using the device_id and packet_id."""
        if (not self.serial_device.serialPort.is_open):
            self.comport_status = 'closed'
        else:
            self.comport_status = 'open'

            packets = self.serial_device.readdata()
            for packet in packets:
                rxdev_id, rxpacket_id, rxdata = self.serial_device.parsepacket(packet)
                if rxdev_id > 0 and rxpacket_id != 0 and rxdata != 0:
                    motor = self.get_motor_by_device_id(rxdev_id)
                    pid = rxpacket_id
                    pdata = rxdata
                    # Basic operations to convert some incoming data forms to correct information
                    if pid == PacketID.POSITION:
                        data = pdata[0] * 360 / 6.2831853
                    elif pid == PacketID.VELOCITY:
                        data = pdata[0] * 360 / 6.2831853
                    elif pid == PacketID.OPENLOOP:
                        data = pdata[0] * 100
                    elif pid == PacketID.MODE:
                        data = pdata[0]
                    elif pid == PacketID.CURRENT:
                        data = pdata[0]
                    elif (pid == PacketID.VELOCITY_GAIN
                            or pid == PacketID.POSITION_GAIN
                            or pid == PacketID.CURRENT_GAIN):
                        data = {'KP': pdata[0], 'KI': pdata[1], 'KD': pdata[2], 'KF': pdata[3]}
                    elif (pid == PacketID.POSITION_LIMIT
                            or pid == PacketID.CURRENT_LIMIT
                            or pid == PacketID.VELOCITY_LIMIT):
                        data = {'min': pdata[1], 'max': pdata[0]}
                        print('VEL/CUR/POS LIMIT received. min:', data['min'], 'max:', data['max'])
                    else:
                        data = pdata[0]
                    if motor:
                        print('Device data received:', 'Device ID:', motor['deviceID'], 'Packet ID:', pid, 'Data: ', data)
                        motor[pid] = data
                    else:
                        print('Device ID: ', rxdev_id, 'Packet ID: ', pid, 'Data: ', data, 'No motor set up for this device id')
