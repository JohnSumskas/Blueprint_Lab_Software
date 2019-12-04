import inspect


import RS1_hardware
from RS1_hardware import PacketID

class PacketPrototype():
    packet_id = 0
    data_type = None
    uses_no_change = False
    data_comments = None
    instances = []

    def __init__(self, packet_id, data_type=None, data_names=['value'], alt_data_names=None, data_comments=None, uses_no_change=False):
        PacketPrototype.instances.append(self)
        self.rx_data = {}
        self.data_names = []
        self.alt_data_names = None
        self.packet_id = packet_id
        self.callbacks = []
        if data_type in [int, float]:
            self.data_type = data_type
        if isinstance(data_names, list):
            self.data_names = data_names
            self.alt_data_names = alt_data_names
        for d in self.data_names:
            self.rx_data[d] = None
        self.uses_no_change = uses_no_change
        if data_comments is None:
            self.data_comments = None
        else:
            len_comments = len(data_names) if len(data_names)>=len(data_comments) else len(data_comments)
            self.data_comments = data_comments[:len_comments]




class Packets:

    def __init__(self):
        self.MODE = PacketPrototype(PacketID.MODE, data_type=int, data_comments=['Axis Mode: Position, Velocity, etc...'])

        self.VELOCITY = PacketPrototype(PacketID.VELOCITY, data_type=float, data_comments=['Axis Velocity (radians/s or mm/s)'])
        self.POSITION = PacketPrototype(PacketID.POSITION, data_type=float, data_comments=['Axis Position (radians or mm)'])
        self.OPENLOOP = PacketPrototype(PacketID.OPENLOOP, data_type=float,
                                   data_comments=['Axis Openloop Control (% in range 0.0-1.0)'])
        self.CURRENT = PacketPrototype(PacketID.CURRENT, data_type=float, data_comments=['Axis Current (mA)'])

        self.RELATIVE_POSITION = PacketPrototype(0x0E, data_type=float)

        self.VELOCITY_DEMAND_INNER = PacketPrototype(0x07, data_type=float)
        self.POSITION_DEMAND_INNER = PacketPrototype(0x08, data_type=float)
        self.CURRENT_DEMAND_DIRECT = PacketPrototype(0x09, data_type=float)

        self.SUPPLYVOLTAGE = PacketPrototype(PacketID.SUPPLYVOLTAGE, data_type=float)
        self.TEMPERATURE = PacketPrototype(PacketID.TEMPERATURE, data_type=float)
        self.REQUEST_PACKET = PacketPrototype(PacketID.REQUEST_PACKET, data_type=int,
                                         data_names=["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"])
        self.SERIAL_NUMBER = PacketPrototype(PacketID.SERIAL_NUMBER, data_type=float)
        self.MODEL_NUMBER = PacketPrototype(PacketID.MODEL_NUMBER, data_type=float)
        self.VERSION = PacketPrototype(PacketID.VERSION, data_type=float)

        self.DEVICE_ID = PacketPrototype(PacketID.DEVICE_ID, data_type=int, data_comments=['Axis Device ID'])

        self.INTERNAL_HUMIDITY = PacketPrototype(PacketID.INTERNAL_HUMIDITY, data_type=float)
        self.INTERNAL_TEMPERATURE = PacketPrototype(PacketID.INTERNAL_TEMPERATURE, data_type=float)

        self.DEVICE_TYPE = PacketPrototype(PacketID.DEVICE_TYPE, data_type=int)
        self.HARDWARE_STATUS = PacketPrototype(PacketID.HARDWARE_STATUS, data_type=int)

        self.RUN_TIME = PacketPrototype(PacketID.RUN_TIME, data_type=float)

        self.COMS_PROTOCOL = PacketPrototype(PacketID.COMS_PROTOCOL, data_type=int)

        self.HEARTBEAT_FREQUENCY_SET = PacketPrototype(PacketID.HEARTBEAT_FREQUENCY_SET, data_type=int,
                                                  data_comments=['Axis Heartbeat Frequency (Hz)'])
        self.HEARTBEAT_SET = PacketPrototype(PacketID.HEARTBEAT_SET, data_type=int,
                                        data_names=["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"])

        self.SAVE = PacketPrototype(PacketID.SAVE, data_type=int)
        self.LOAD = PacketPrototype(PacketID.LOAD, data_type=None)
        self.SET_DEFAULTS = PacketPrototype(PacketID.SET_DEFAULTS, data_type=None)
        self.FORMAT = PacketPrototype(PacketID.FORMAT, data_type=None)
        self.CHANGE_PAGE = PacketPrototype(PacketID.CHANGE_PAGE, data_type=None)

        self.CURRENT_LIMIT = PacketPrototype(PacketID.CURRENT_LIMIT, data_type=float, data_names=['max', 'min'],
                                        data_comments=['Maximum Current (mA)', 'Minimum Current (mA)'])
        self.VELOCITY_LIMIT = PacketPrototype(PacketID.VELOCITY_LIMIT, data_type=float, data_names=['max', 'min'])
        self.POSITION_LIMIT = PacketPrototype(PacketID.POSITION_LIMIT, data_type=float, data_names=['max', 'min'])
        self.POSITION_GAIN = PacketPrototype(PacketID.POSITION_GAIN, data_type=float,
                                        data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.VELOCITY_GAIN = PacketPrototype(PacketID.VELOCITY_GAIN, data_type=float,
                                        data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.CURRENT_GAIN = PacketPrototype(PacketID.CURRENT_GAIN, data_type=float,
                                       data_names=['KP', 'KI', 'KD', 'KF', 'MI'])

        self.VELOCITY_LIMIT_INNER = PacketPrototype(PacketID.VELOCITY_LIMIT_INNER, data_type=float,
                                               data_names=['max', 'min'])
        self.POSITION_GAINS_INNER = PacketPrototype(PacketID.POSITION_GAINS_INNER, data_type=float,
                                               data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.VELOCITY_GAINS_INNER = PacketPrototype(PacketID.VELOCITY_GAINS_INNER, data_type=float,
                                               data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.CURRENT_GAINS_DIRECT = PacketPrototype(PacketID.CURRENT_GAINS_DIRECT, data_type=float,
                                               data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.VELOCITY_INNER_PARAMETERS = PacketPrototype(PacketID.VELOCITY_INNER_PARAMETERS, data_type=float,
                                                    data_names=['scale', 'offset', 'alpha', 'beta'])

        self.AUTO_LIMIT_CURRENT_DEMAND = PacketPrototype(PacketID.AUTO_LIMIT_CURRENT_DEMAND, data_type=float)

        self.POSITION_PARAMETERS = PacketPrototype(PacketID.POSITION_PARAMETERS, data_type=float,
                                              data_names=['scale', 'offset', 'alpha', 'beta'])
        self.VELOCITY_PARAMETERS = PacketPrototype(PacketID.VELOCITY_PARAMETERS, data_type=float,
                                              data_names=['scale', 'offset', 'alpha', 'beta'])
        self.CURRENT_PARAMETERS = PacketPrototype(PacketID.CURRENT_PARAMETERS, data_type=float,
                                             data_names=['scale', 'offset', 'alpha', 'beta'])
        self.INPUT_VOLTAGE_PARAMETERS = PacketPrototype(PacketID.INPUT_VOLTAGE_PARAMETERS, data_type=float,
                                                   data_names=['scale', 'offset', 'alpha', 'beta'])
        self.MOTOR_PARAMETERS = PacketPrototype(PacketID.MOTOR_PARAMETERS, data_type=float,
                                           data_names=['voltage', 'current', 'resistance', 'direction'])
        self.ICMU_INNER_PARAMETERS = PacketPrototype(PacketID.ICMU_INNER_PARAMETERS, data_type=float,
                                                data_names=['direction', 'set_zero'])
        self.MAX_ACCELERATION = PacketPrototype(PacketID.MAX_ACCELERATION, data_type=float)
        self.CURRENT_HOLD_THRESHOLD = PacketPrototype(PacketID.CURRENT_HOLD_THRESHOLD, data_type=float)
        self.COMPLIANCE_GAIN = PacketPrototype(PacketID.COMPLIANCE_GAIN, data_type=float)
        self.COMPLIANCE_PARAMETERS = PacketPrototype(PacketID.COMPLIANCE_PARAMETERS, data_type=float,
                                                data_names=['wind_down_alpha', 'backdrive_efficiency', 'trigger_alpha', 'beta'],
                                                alt_data_names=[['compliance_gain'], [], ['alpha'], []])

        # 3 bytes for each version packet
        # [major, submaj, minor]
        self.ELECTRICAL_VERSION = PacketPrototype(PacketID.ELECTRICAL_VERSION, data_type=int,
                                             data_names=['major', 'submajor', 'min'])
        self.MECHANICAL_VERSION = PacketPrototype(PacketID.MECHANICAL_VERSION, data_type=int,
                                             data_names=['major', 'submajor', 'min'])
        self.SOFTWARE_VERSION = PacketPrototype(PacketID.SOFTWARE_VERSION, data_type=int,
                                           data_names=['major', 'submajor', 'min'])

        self.BOOTLOADER = PacketPrototype(PacketID.BOOTLOADER, data_type=None)

        self.KM_CONFIGURATION = PacketPrototype(PacketID.KM_CONFIGURATION, data_type=int,
                                           data_names=['enable', 'obstacles', 'orientation', 'frame'],
                                           uses_no_change=True)
        self.KM_END_POS = PacketPrototype(PacketID.KM_END_POS, data_type=float, data_names=['x', 'y', 'z'])
        self.KM_END_VEL = PacketPrototype(PacketID.KM_END_VEL, data_type=float, data_names=['x', 'y', 'z'])

        self.KM_BOX_OBSTACLE_00 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_00, data_type=float,
                                             data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
        self.KM_BOX_OBSTACLE_01 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_01, data_type=float,
                                             data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
        self.KM_BOX_OBSTACLE_02 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_02, data_type=float,
                                             data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
        self.KM_BOX_OBSTACLE_03 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_03, data_type=float,
                                             data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
        self.KM_BOX_OBSTACLE_04 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_04, data_type=float,
                                             data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
        self.KM_BOX_OBSTACLE_05 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_05, data_type=float,
                                             data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])

        self.KM_CYLINDER_OBSTACLE_00 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_00, data_type=float,
                                                  data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
        self.KM_CYLINDER_OBSTACLE_01 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_01, data_type=float,
                                                  data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
        self.KM_CYLINDER_OBSTACLE_02 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_02, data_type=float,
                                                  data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
        self.KM_CYLINDER_OBSTACLE_03 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_03, data_type=float,
                                                  data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
        self.KM_CYLINDER_OBSTACLE_04 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_04, data_type=float,
                                                  data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
        self.KM_CYLINDER_OBSTACLE_05 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_05, data_type=float,
                                                  data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])

        self.KM_FLOAT_PARAMETERS = PacketPrototype(PacketID.KM_FLOAT_PARAMETERS, data_type=float,
                                              data_names=['m_zero', 'lambda_translate', 'lambda_rotate',
                                                          'collision_fwd_time_steps', 'self_collision_radius',
                                                          'end_eff_collision_tolerance'])

        # KM_JOINT_STATE = 0xB2
        # KM_JOINT_STATE_REQUEST = 0xB3

        self.KM_DH_PARAMETERS_0 = PacketPrototype(PacketID.KM_DH_PARAMETERS_0, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
        self.KM_DH_PARAMETERS_1 = PacketPrototype(PacketID.KM_DH_PARAMETERS_1, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
        self.KM_DH_PARAMETERS_2 = PacketPrototype(PacketID.KM_DH_PARAMETERS_2, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
        self.KM_DH_PARAMETERS_3 = PacketPrototype(PacketID.KM_DH_PARAMETERS_3, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
        self.KM_DH_PARAMETERS_4 = PacketPrototype(PacketID.KM_DH_PARAMETERS_4, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
        self.KM_DH_PARAMETERS_5 = PacketPrototype(PacketID.KM_DH_PARAMETERS_5, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
        self.KM_DH_PARAMETERS_6 = PacketPrototype(PacketID.KM_DH_PARAMETERS_6, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
        self.KM_DH_PARAMETERS_7 = PacketPrototype(PacketID.KM_DH_PARAMETERS_7, data_type=float,
                                             data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])

        self.KM_POS_LIMIT_TRANSLATE = PacketPrototype(PacketID.KM_POS_LIMIT_TRANSLATE, data_type=float,
                                                 data_names=['max', 'min'])
        self.KM_VEL_LIMIT_TRANSLATE = PacketPrototype(PacketID.KM_VEL_LIMIT_TRANSLATE, data_type=float,
                                                 data_names=['max', 'min'])
        self.KM_POS_LIMIT_YAW = PacketPrototype(PacketID.KM_POS_LIMIT_YAW, data_type=float,
                                           data_names=['max', 'min'])
        self.KM_POS_LIMIT_PITCH = PacketPrototype(PacketID.KM_POS_LIMIT_PITCH, data_type=float,
                                             data_names=['max', 'min'])
        self.KM_POS_LIMIT_ROLL = PacketPrototype(PacketID.KM_POS_LIMIT_ROLL, data_type=float,
                                            data_names=['max', 'min'])
        self.KM_VEL_LIMIT_ROTATE = PacketPrototype(PacketID.KM_VEL_LIMIT_ROTATE, data_type=float,
                                              data_names=['max', 'min'])

        self.KM_POS_GAINS_TRANSLATE = PacketPrototype(PacketID.KM_POS_GAINS_TRANSLATE, data_type=float,
                                                 data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.KM_VEL_GAINS_TRANSLATE = PacketPrototype(PacketID.KM_VEL_GAINS_TRANSLATE, data_type=float,
                                                 data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.KM_POS_GAINS_ROTATE = PacketPrototype(PacketID.KM_POS_GAINS_ROTATE, data_type=float,
                                              data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.KM_VEL_GAINS_ROTATE = PacketPrototype(PacketID.KM_VEL_GAINS_ROTATE, data_type=float,
                                              data_names=['KP', 'KI', 'KD', 'KF', 'MI'])

        self.KM_JOINT_STATE_REQUEST = PacketPrototype(PacketID.KM_JOINT_STATE_REQUEST, data_type=int)

        self.ENCODER = PacketPrototype(PacketID.ENCODER, data_type=int,
                                  data_names=['address1', 'data1', 'address2', 'data2'])
        self.ENCODER_PARAMETERS = PacketPrototype(PacketID.ENCODER_PARAMETERS, data_type=int,
                                             data_names=['direction', 'set_zero'])

        # RS2 Specific - Needs refining and sub-parameters
        self.VELOCITY_DEMAND_INNER = PacketPrototype(PacketID.VELOCITY_DEMAND_INNER, data_type=float)
        self.POSITION_DEMAND_INNER = PacketPrototype(PacketID.POSITION_DEMAND_INNER, data_type=float)
        self.CURRENT_DEMAND_DIRECT = PacketPrototype(PacketID.CURRENT_DEMAND_DIRECT, data_type=float)
        self.ICMU_INNER_PARAMETERS = PacketPrototype(PacketID.ICMU_INNER_PARAMETERS, data_type=float,
                                                data_names=['direction', 'set_zero'])
        # ICMU_PARAMETERS = PacketPrototype(PacketID.ICMU_PARAMETERS, data_type=float)
        self.VELOCITY_LIMIT_INNER = PacketPrototype(PacketID.VELOCITY_LIMIT_INNER, data_type=float,
                                               data_names=['max', 'min'])
        self.POSITION_GAINS_INNER = PacketPrototype(PacketID.POSITION_GAINS_INNER, data_type=float,
                                               data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.VELOCITY_GAINS_INNER = PacketPrototype(PacketID.VELOCITY_GAINS_INNER, data_type=float,
                                               data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.CURRENT_GAINS_DIRECT = PacketPrototype(PacketID.CURRENT_GAINS_DIRECT, data_type=float,
                                               data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
        self.VELOCITY_INNER_PARAMETERS = PacketPrototype(PacketID.VELOCITY_INNER_PARAMETERS, data_type=float,
                                                    data_names=['scale', 'offset', 'alpha', 'beta'])

        self.MODE_SETTINGS = PacketPrototype(PacketID.MODE_SETTINGS, data_type=int,
                                        data_names=['current_hold', 'compliance'],
                                        uses_no_change=True)

        self.BOOTLOADER_STM = PacketPrototype(PacketID.BOOTLOADER_STM, data_type=None)

        self.TEST_PACKET = PacketPrototype(PacketID.TEST_PACKET, data_type=int)
        self.RESET = PacketPrototype(PacketID.RESET, data_type=None)

    def get_by_id(self, packet_id):
        for p in self.get_all():
            if p.packet_id == packet_id:
                return p
        else:
            return None

    def get_all(self):
        attrlist = inspect.getmembers(self)
        # print(__name__, 'get_all()', attrlist)
        packets_list = []
        for member in attrlist:
            # print(__name__, 'type(member)', member, type(member[1]))
            if isinstance(member[1], PacketPrototype):
                packets_list.append(member[1])
        return packets_list

    def get_float_packet_ids(self):
        all_members = self.get_all()
        float_packet_id_list = []
        for member in all_members:
            if member.data_type == float:
                float_packet_id_list.append(member.packet_id)
        return float_packet_id_list

    MODE = PacketPrototype(PacketID.MODE, data_type=int, data_comments=['Axis Mode: Position, Velocity, etc...'])

    VELOCITY = PacketPrototype(PacketID.VELOCITY, data_type=float, data_comments=['Axis Velocity (radians/s)'])
    POSITION = PacketPrototype(PacketID.POSITION, data_type=float, data_comments=['Axis Position (radians)'])
    OPENLOOP = PacketPrototype(PacketID.OPENLOOP, data_type=float, data_comments=['Axis Openloop Control (radians/s)'])
    CURRENT = PacketPrototype(PacketID.CURRENT, data_type=float, data_comments=['Axis Current (mA)'])

    RELATIVE_POSITION = PacketPrototype(0x0E, data_type=float)

    VELOCITY_DEMAND_INNER = PacketPrototype(0x07, data_type=float)
    POSITION_DEMAND_INNER = PacketPrototype(0x08, data_type=float)
    CURRENT_DEMAND_DIRECT = PacketPrototype(0x09, data_type=float)

    SUPPLYVOLTAGE = PacketPrototype(PacketID.SUPPLYVOLTAGE, data_type=float)
    TEMPERATURE = PacketPrototype(PacketID.TEMPERATURE, data_type=float)
    REQUEST_PACKET = PacketPrototype(PacketID.REQUEST_PACKET, data_type=int,
                                     data_names=["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"])
    SERIAL_NUMBER = PacketPrototype(PacketID.SERIAL_NUMBER, data_type=float)
    MODEL_NUMBER = PacketPrototype(PacketID.MODEL_NUMBER, data_type=float)
    VERSION = PacketPrototype(PacketID.VERSION, data_type=float)

    DEVICE_ID = PacketPrototype(PacketID.DEVICE_ID, data_type=int, data_comments=['Axis Device ID'])

    INTERNAL_HUMIDITY = PacketPrototype(PacketID.INTERNAL_HUMIDITY, data_type=float)
    INTERNAL_TEMPERATURE = PacketPrototype(PacketID.INTERNAL_TEMPERATURE, data_type=float)

    DEVICE_TYPE = PacketPrototype(PacketID.DEVICE_TYPE, data_type=int)
    HARDWARE_STATUS = PacketPrototype(PacketID.HARDWARE_STATUS, data_type=int)

    RUN_TIME = PacketPrototype(PacketID.RUN_TIME, data_type=float)

    COMS_PROTOCOL = PacketPrototype(PacketID.COMS_PROTOCOL, data_type=int)

    HEARTBEAT_FREQUENCY_SET = PacketPrototype(PacketID.HEARTBEAT_FREQUENCY_SET, data_type=int, data_comments=['Axis Heartbeat Frequency (Hz)'])
    HEARTBEAT_SET = PacketPrototype(PacketID.HEARTBEAT_SET, data_type=int,
                                    data_names=["a", "b", "c", "d", "e", "f", "g", "h", "i", "j"])

    SAVE = PacketPrototype(PacketID.SAVE, data_type=int)
    LOAD = PacketPrototype(PacketID.LOAD, data_type=None)
    SET_DEFAULTS = PacketPrototype(PacketID.SET_DEFAULTS, data_type=None)
    FORMAT = PacketPrototype(PacketID.FORMAT, data_type=None)
    CHANGE_PAGE = PacketPrototype(PacketID.CHANGE_PAGE, data_type=None)

    CURRENT_LIMIT = PacketPrototype(PacketID.CURRENT_LIMIT, data_type=float, data_names=['max', 'min'], data_comments=['Maximum Current (mA)', 'Minimum Current (mA)'])
    VELOCITY_LIMIT = PacketPrototype(PacketID.VELOCITY_LIMIT, data_type=float, data_names=['max', 'min'])
    POSITION_LIMIT = PacketPrototype(PacketID.POSITION_LIMIT, data_type=float, data_names=['max', 'min'])
    POSITION_GAIN = PacketPrototype(PacketID.POSITION_GAIN, data_type=float, data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    VELOCITY_GAIN = PacketPrototype(PacketID.VELOCITY_GAIN, data_type=float, data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    CURRENT_GAIN = PacketPrototype(PacketID.CURRENT_GAIN, data_type=float, data_names=['KP', 'KI', 'KD', 'KF', 'MI'])

    VELOCITY_LIMIT_INNER = PacketPrototype(PacketID.VELOCITY_LIMIT_INNER, data_type=float,
                                           data_names=['max', 'min'])
    POSITION_GAINS_INNER = PacketPrototype(PacketID.POSITION_GAINS_INNER, data_type=float,
                                           data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    VELOCITY_GAINS_INNER = PacketPrototype(PacketID.VELOCITY_GAINS_INNER, data_type=float,
                                           data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    CURRENT_GAINS_DIRECT = PacketPrototype(PacketID.CURRENT_GAINS_DIRECT, data_type=float,
                                           data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    VELOCITY_INNER_PARAMETERS = PacketPrototype(PacketID.VELOCITY_INNER_PARAMETERS, data_type=float,
                                                data_names=['scale', 'offset', 'alpha', 'beta'])


    AUTO_LIMIT_CURRENT_DEMAND = PacketPrototype(PacketID.AUTO_LIMIT_CURRENT_DEMAND, data_type=float)

    POSITION_PARAMETERS = PacketPrototype(PacketID.POSITION_PARAMETERS, data_type=float,
                                          data_names=['scale', 'offset', 'alpha', 'beta'])
    VELOCITY_PARAMETERS = PacketPrototype(PacketID.VELOCITY_PARAMETERS, data_type=float,
                                          data_names=['scale', 'offset', 'alpha', 'beta'])
    CURRENT_PARAMETERS = PacketPrototype(PacketID.CURRENT_PARAMETERS, data_type=float,
                                         data_names=['scale', 'offset', 'alpha', 'beta'])
    INPUT_VOLTAGE_PARAMETERS = PacketPrototype(PacketID.INPUT_VOLTAGE_PARAMETERS, data_type=float,
                                               data_names=['scale', 'offset', 'alpha', 'beta'])
    MOTOR_PARAMETERS = PacketPrototype(PacketID.MOTOR_PARAMETERS, data_type=float,
                                       data_names=['voltage', 'current', 'resistance', 'direction'])
    ICMU_INNER_PARAMETERS = PacketPrototype(PacketID.ICMU_INNER_PARAMETERS, data_type=float,
                                            data_names=['direction', 'set_zero'])
    MAX_ACCELERATION = PacketPrototype(PacketID.MAX_ACCELERATION, data_type=float)
    CURRENT_HOLD_THRESHOLD = PacketPrototype(PacketID.CURRENT_HOLD_THRESHOLD, data_type=float)
    COMPLIANCE_GAIN = PacketPrototype(PacketID.COMPLIANCE_GAIN, data_type=float)
    COMPLIANCE_PARAMETERS = PacketPrototype(PacketID.COMPLIANCE_PARAMETERS, data_type=float,
                                            data_names=['wind_down_alpha', 'backdrive_efficiency', 'trigger_alpha', 'beta'],
                                            alt_data_names=[['compliance_gain'], [], ['alpha'], []])

    # 3 bytes for each version packet
    # [major, submaj, minor]
    ELECTRICAL_VERSION = PacketPrototype(PacketID.ELECTRICAL_VERSION, data_type=int,
                                         data_names=['major', 'submajor', 'min'])
    MECHANICAL_VERSION = PacketPrototype(PacketID.MECHANICAL_VERSION, data_type=int,
                                         data_names=['major', 'submajor', 'min'])
    SOFTWARE_VERSION = PacketPrototype(PacketID.SOFTWARE_VERSION, data_type=int,
                                       data_names=['major', 'submajor', 'min'])

    BOOTLOADER = PacketPrototype(PacketID.BOOTLOADER, data_type=None)

    KM_CONFIGURATION = PacketPrototype(PacketID.KM_CONFIGURATION, data_type=int,
                                       data_names=['enable', 'obstacles', 'orientation', 'frame'],
                                       uses_no_change=True)
    KM_END_POS = PacketPrototype(PacketID.KM_END_POS, data_type=float, data_names=['x', 'y', 'z'])
    KM_END_VEL = PacketPrototype(PacketID.KM_END_VEL, data_type=float, data_names=['x', 'y', 'z'])

    KM_BOX_OBSTACLE_00 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_00, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
    KM_BOX_OBSTACLE_01 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_01, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
    KM_BOX_OBSTACLE_02 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_02, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
    KM_BOX_OBSTACLE_03 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_03, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
    KM_BOX_OBSTACLE_04 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_04, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
    KM_BOX_OBSTACLE_05 = PacketPrototype(PacketID.KM_BOX_OBSTACLE_05, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])

    KM_CYLINDER_OBSTACLE_00 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_00, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
    KM_CYLINDER_OBSTACLE_01 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_01, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
    KM_CYLINDER_OBSTACLE_02 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_02, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
    KM_CYLINDER_OBSTACLE_03 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_03, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
    KM_CYLINDER_OBSTACLE_04 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_04, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])
    KM_CYLINDER_OBSTACLE_05 = PacketPrototype(PacketID.KM_CYLINDER_OBSTACLE_05, data_type=float,
                                         data_names=['x1', 'y1', 'z1', 'x2', 'y2', 'z2', 'radius'])

    KM_FLOAT_PARAMETERS = PacketPrototype(PacketID.KM_FLOAT_PARAMETERS, data_type=float,
                                          data_names=['m_zero', 'lambda_translate', 'lambda_rotate',
                                                      'collision_fwd_time_steps','self_collision_radius',
                                                      'end_eff_collision_tolerance'])

    # KM_JOINT_STATE = 0xB2
    # KM_JOINT_STATE_REQUEST = 0xB3

    KM_DH_PARAMETERS_0 = PacketPrototype(PacketID.KM_DH_PARAMETERS_0, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
    KM_DH_PARAMETERS_1 = PacketPrototype(PacketID.KM_DH_PARAMETERS_1, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
    KM_DH_PARAMETERS_2 = PacketPrototype(PacketID.KM_DH_PARAMETERS_2, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
    KM_DH_PARAMETERS_3 = PacketPrototype(PacketID.KM_DH_PARAMETERS_3, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
    KM_DH_PARAMETERS_4 = PacketPrototype(PacketID.KM_DH_PARAMETERS_4, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
    KM_DH_PARAMETERS_5 = PacketPrototype(PacketID.KM_DH_PARAMETERS_5, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
    KM_DH_PARAMETERS_6 = PacketPrototype(PacketID.KM_DH_PARAMETERS_6, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])
    KM_DH_PARAMETERS_7 = PacketPrototype(PacketID.KM_DH_PARAMETERS_7, data_type=float,
                                         data_names=['d', 'a', 'alpha', 'theta_offset', 'theta_min', 'theta_max'])

    KM_POS_LIMIT_TRANSLATE = PacketPrototype(PacketID.KM_POS_LIMIT_TRANSLATE, data_type=float,
                                             data_names=['max', 'min'])
    KM_VEL_LIMIT_TRANSLATE = PacketPrototype(PacketID.KM_VEL_LIMIT_TRANSLATE, data_type=float,
                                             data_names=['max', 'min'])
    KM_POS_LIMIT_YAW = PacketPrototype(PacketID.KM_POS_LIMIT_YAW, data_type=float,
                                       data_names=['max', 'min'])
    KM_POS_LIMIT_PITCH = PacketPrototype(PacketID.KM_POS_LIMIT_PITCH, data_type=float,
                                         data_names=['max', 'min'])
    KM_POS_LIMIT_ROLL = PacketPrototype(PacketID.KM_POS_LIMIT_ROLL, data_type=float,
                                        data_names=['max', 'min'])
    KM_VEL_LIMIT_ROTATE = PacketPrototype(PacketID.KM_VEL_LIMIT_ROTATE, data_type=float,
                                          data_names=['max', 'min'])

    KM_POS_GAINS_TRANSLATE = PacketPrototype(PacketID.KM_POS_GAINS_TRANSLATE, data_type=float,
                                             data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    KM_VEL_GAINS_TRANSLATE = PacketPrototype(PacketID.KM_VEL_GAINS_TRANSLATE, data_type=float,
                                             data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    KM_POS_GAINS_ROTATE = PacketPrototype(PacketID.KM_POS_GAINS_ROTATE, data_type=float,
                                          data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    KM_VEL_GAINS_ROTATE = PacketPrototype(PacketID.KM_VEL_GAINS_ROTATE, data_type=float,
                                          data_names=['KP', 'KI', 'KD', 'KF', 'MI'])

    KM_JOINT_STATE_REQUEST = PacketPrototype(PacketID.KM_JOINT_STATE_REQUEST, data_type=int)

    ENCODER = PacketPrototype(PacketID.ENCODER, data_type=int,
                              data_names=['address1', 'data1', 'address2', 'data2'])
    ENCODER_PARAMETERS = PacketPrototype(PacketID.ENCODER_PARAMETERS, data_type=int,
                                         data_names=['direction', 'set_zero'])

    # RS2 Specific - Needs refining and sub-parameters
    VELOCITY_DEMAND_INNER = PacketPrototype(PacketID.VELOCITY_DEMAND_INNER, data_type=float)
    POSITION_DEMAND_INNER = PacketPrototype(PacketID.POSITION_DEMAND_INNER, data_type=float)
    CURRENT_DEMAND_DIRECT = PacketPrototype(PacketID.CURRENT_DEMAND_DIRECT, data_type=float)
    ICMU_INNER_PARAMETERS = PacketPrototype(PacketID.ICMU_INNER_PARAMETERS, data_type=float,
                                            data_names=['direction', 'set_zero'])
    # ICMU_PARAMETERS = PacketPrototype(PacketID.ICMU_PARAMETERS, data_type=float)
    VELOCITY_LIMIT_INNER = PacketPrototype(PacketID.VELOCITY_LIMIT_INNER, data_type=float,
                                           data_names=['max', 'min'])
    POSITION_GAINS_INNER = PacketPrototype(PacketID.POSITION_GAINS_INNER, data_type=float,
                                           data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    VELOCITY_GAINS_INNER = PacketPrototype(PacketID.VELOCITY_GAINS_INNER, data_type=float,
                                           data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    CURRENT_GAINS_DIRECT = PacketPrototype(PacketID.CURRENT_GAINS_DIRECT, data_type=float,
                                           data_names=['KP', 'KI', 'KD', 'KF', 'MI'])
    VELOCITY_INNER_PARAMETERS = PacketPrototype(PacketID.VELOCITY_INNER_PARAMETERS, data_type=float,
                                                data_names=['scale', 'offset', 'alpha', 'beta'])

    MODE_SETTINGS = PacketPrototype(PacketID.MODE_SETTINGS, data_type=int,
                                    data_names=['current_hold', 'compliance'],
                                    uses_no_change=True)

    BOOTLOADER_STM = PacketPrototype(PacketID.BOOTLOADER_STM, data_type=None)

    TEST_PACKET = PacketPrototype(PacketID.TEST_PACKET, data_type=int)
    RESET = PacketPrototype(PacketID.RESET, data_type=None)

