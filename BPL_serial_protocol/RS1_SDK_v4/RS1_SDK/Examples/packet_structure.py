
from construct import *

# packet_crc = Struct('PACKET_CRC', Int32ul('CRC'))

packet_format = Struct('packet_format',
    Byte('data_length_indicator'),  ## DATA LENGTH TO BE DELETED AFTER BUILDING
    Array(lambda arr: arr['data_length_indicator'], 
    Byte('data')),    ## DATA
    Byte('packet_id'),                               ## PACKET ID
    Byte('device_id'),                                      ## BOARD/DEVICE ID
    Byte('packet_length'),  ## PACKET LENGTH
    Byte('packet_crc'),  ## CRC for later
    # Byte('terminator')                                      ## TERMINATOR
)

