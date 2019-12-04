"""Serial Device file. 
    Packet encoding and decoding, plus transmission.
"""

# from kivy.app import App
import serial,struct,re, binascii
import platform

# Get python version (python 2 or 3 - other versions not supported)
py_version = int(platform.python_version()[0])

# Byte-stuffing functions for serial transmission
if py_version == 2:
    from cobs_py2 import cobs
else:
    from cobs import cobs

from packet_structure import packet_format, Container

# Cyclic Redundancy Check (CRC) functions for checking transmission was correct
if py_version == 2:
	from crcmod_py2 import crcmod
else:
	from crcmod import crcmod

from functools import partial
from construct import *
from RS1_hardware import PacketID, Mode, FLOAT_PACKETS
from array import array
import array



 
def convert_to_bytes(parameter):
    """Convert numeric property velocity into bytes."""
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



class SerialDevice():
    """SerialDevice class: Primary class for managing serial connection,
        encoding outgoing data, and decoding incoming data.
    """
    port=None
    baud=None
    serialPort=serial.Serial()
    txPacketId=0
    txData=0
    txDeviceId=0

    rxPacketId=0
    rxData=[0]
    rxDeviceId=0

    incomplete_pckt=b''

    packets=b''
    pckt=b''

    parse_packet_errors=0

    verbose = False # set True for print statements


    def __init__(self):
        """Cyclic Redundancy Check (CRC) function creation to enable 
        data bit coherency checking.
        By default, 0x14D is the CRC encoding polynomial used by our products
        """
        self.crc8_func = crcmod.mkCrcFun(0x14D, initCrc=0xFF, xorOut=0xFF)

    def open(self):
        """Attempt to open serial port, otherwise throw exception."""
        print('Serial device settings (port, baudrate):', self.port, str(self.baud))
        self.serialPort.baudrate = self.baud
        self.serialPort.port = self.port

        # Open serial port
        if self.serialPort.is_open == False:
            try:
                self.serialPort.open()
            except:
                print ("Failed to connect")
                raise Exception('Failed to Connect Error')
                #exit()
        if self.serialPort.is_open:
            print("serial is open")
            return self.serialPort
        else:
            return None

    def close(self):
        if self.serialPort.is_open:
            self.serialPort.close()



    def sendpacket(self, device_id, packet_id, data_in):
        """Convert device_id, packet_id and data to BPL packet and send via serial. Returns the encoded packet."""
        if self.verbose: print('\npre-check', str(device_id), str(packet_id), data_in)
        txData = convert_to_bytes(data_in)
        if self.verbose:
            print(txData)

        datalength = len(txData)

        packet_length = datalength + 4

        txPacket = txData
        txPacket.append(packet_id)
        txPacket.append(device_id)
        txPacket.append(packet_length)
        
        if py_version == 2:
            txPacket_str = str(txPacket)
            crcValue=self.crc8_func(txPacket_str)
            txPacket.append(crcValue)
            txPacket_str = str(txPacket)
            encoded = cobs.encode(txPacket_str)
        else:
            crcValue=self.crc8_func(txPacket)
            txPacket.append(crcValue)
            encoded = cobs.encode(txPacket)

        if self.verbose: print('encoded packet check')
        if self.verbose: print([ "0x%s" % b for b in encoded ])

        ## ADD TERMINATOR BYTE
        packet = encoded + b"\x00"
        if self.serialPort!=None:
            if self.serialPort.is_open:
                if self.verbose: print('send packet check')
                if self.verbose: print([ "0x%s" % b for b in packet ])
                self.serialPort.write(packet) 
        else:
            print("Serial Port is not Open")
        return packet


###_________________________ Receive Function_____________________________###    
    def readdata(self):
        """Read data on the serial bus. Returns complete packets."""
        if self.serialPort!=None:
            if self.serialPort.is_open:
                bytesToRead = 0

                try:
                    bytesToRead = self.serialPort.in_waiting
                except:
                    self.close()

                if bytesToRead:
                    newBytes=self.serialPort.read(bytesToRead)

                    if self.verbose: print('New bytes:', [ " 0x%s" % b for b in newBytes ])
                    buff = self.incomplete_pckt + newBytes
                  
                    if self.verbose: print("buffer received is", buff)

                    packets = re.split(b'\x00', buff)
                    if self.verbose: print(packets)
                   
                    if buff[-1] != b'0x00':
                        self.incomplete_pckt=packets.pop()

                    if self.verbose:
                        for pkt in packets:
                            print(__name__, 'readdata()', [" 0x%s" % b for b in pkt])
                        print('incomplete', [" 0x%s" % b for b in self.incomplete_pckt])
                    return packets

                else:           # no more bytes to read
                    return []

            else:       # pass while waiting for send to finish
                return []

    def parsepacket(self, packet_in):
        """Parses packet data according to packet ID. Must be called for every packet in the packets buffer."""
        if packet_in != b'' and len(packet_in) > 3:
            if py_version == 2: # Python 2 compatibility
                packet_temp = bytes(packet_in)
                packet_in = packet_temp
            try:
                decoded_pckt = cobs.decode(packet_in)
            except Exception as e:   
                print('decode failed', str(e))         	
                return 0,0,0

            decoded_pkt_m2_temp = None
            if py_version == 2: # Python 2 compatibility
                decoded_pkt_m2_temp = int(binascii.hexlify(decoded_pckt[-2]))
            else:
                decoded_pkt_m2_temp = decoded_pckt[-2]

            if decoded_pkt_m2_temp != len(decoded_pckt): # check that length is correct. Generally would only happen on first packet or due to bad transmission
                self.parse_packet_errors += 1
                print(__name__, 'error count:', self.parse_packet_errors, " ####################################################"
                                "PARSEPACKET() incorrect Length, length is:", len(decoded_pckt), [ " 0x%02x" % b for b in decoded_pckt ])
            else:

                crcCheck = self.crc8_func(decoded_pckt[:-1])
                rxCRC=decoded_pckt[-1]
                if py_version == 2: rxCRC = int(binascii.hexlify(rxCRC),16)
                if crcCheck == rxCRC:
                    rxDeviceId=decoded_pckt[-3]
                    rxPacketId=decoded_pckt[-4]
                    rxData=decoded_pckt[:-4]
                    decoded_pckt = b''

                    if self.verbose: print('LIST rxData:', rxData)

                    rxData = bytearray(rxData) # convert list to bytearray

                    rxPacketId_temp = None
                    if py_version == 2:
                        rxPacketId_temp = int(bytearray(rxPacketId)[0])
                    else:
                        rxPacketId_temp = rxPacketId
                    if rxPacketId_temp in FLOAT_PACKETS:
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

                    if self.verbose: print('PARSE float:', rxData)
                    if py_version == 2: # Python 2 compatibility
                        return int(bytearray(rxDeviceId)[0]), int(bytearray(rxPacketId)[0]), rxData
                    else:
                        return rxDeviceId, rxPacketId, rxData

                # print(
                #       "device_id:", rxDeviceId,
                #       "\npacket_id:", rxPacketId,
                #       "\nCRC:", rxCRC,
                #       '\ncrc check: ',crcCheck,
                #       "\ndata:", rxData)

                else:
                    print("CRC Error")
                    # print('crc error packet:', [" 0x%02x" % b for b in packet_in])
        return 0, 0, 0

    
    def parse_packet_to_bytearray(self, packet_in):
        """Returns the raw data without consideration for float or int packet."""
        if packet_in != b'' and len(packet_in) > 3:
            decoded_pckt = cobs.decode(packet_in)  ## Cobs decode
            try:
                decoded_pckt = cobs.decode(packet_in)
            except:            	
                return 0,0,0

            decoded_pkt_m2_temp = None
            if py_version == 2:
                decoded_pkt_m2_temp = int(binascii.hexlify(decoded_pckt[-2]))
            else:
                decoded_pkt_m2_temp = decoded_pckt[-2]

            if decoded_pkt_m2_temp != len(decoded_pckt): # check that length is correct. Generally would only happen on first packet or due to bad transmission
                self.parse_packet_errors += 1
                print(__name__, 'error count:', self.parse_packet_errors, " ####################################################"
                                "PARSEPACKET() incorrect Length, length is:", len(decoded_pckt), [ " 0x%02x" % b for b in decoded_pckt ])
            else:
                crcCheck = self.crc8_func(decoded_pckt[:-1])
                rxCRC = decoded_pckt[-1]
                if crcCheck == rxCRC:
                    rxDeviceId = decoded_pckt[-3]
                    rxPacketId = decoded_pckt[-4]
                    rxData = decoded_pckt[:-4]
                    decoded_pckt = b''

                    if self.verbose: print('LIST rxData:', rxData)

                    rxData = bytearray(rxData)  # convert list to bytearray

                    return rxDeviceId, rxPacketId, rxData

                # print(
                #       "device_id:", rxDeviceId,
                #       "\npacket_id:", rxPacketId,
                #       "\nCRC:", rxCRC,
                #       '\ncrc check: ',crcCheck,
                #       "\ndata:", rxData)

                else:
                    print("CRC Error", [" 0x%02x" % b for b in packet_in])
        return None, None, None

    @staticmethod
    def bytearray_to_float(barray):
        data_to_process = barray
        rxData = []
        for i in range(int(len(data_to_process) / 4)):
            this_float_byte = data_to_process[0:4]
            data_to_process = data_to_process[4:]
            rxData.append(struct.unpack("f", this_float_byte)[0])
        return rxData
