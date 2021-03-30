import os, sys
import glob
from dynamixel_sdk import *

def _get_available_ports():
    """ Tries to find the available serial ports on your system. """
    if platform.system() == 'Darwin':
        return glob.glob('/dev/tty.usb*')

    elif platform.system() == 'Linux':
        return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyAMA*')

    elif sys.platform.lower() == 'cygwin':
        return glob.glob('/dev/com*')

    elif platform.system() == 'Windows':
        import winreg
        import itertools

        ports = []
        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)

        for i in itertools.count():
            try:
                ports.append(str(winreg.EnumValue(key, i)[1]))
            except WindowsError:
                return ports
    else:
        raise EnvironmentError('{} is an unsupported platform, cannot find serial ports!'.format(platform.system()))
    return []

def get_available_ports():
    ports = _get_available_ports()
    return ports


class DxlIO():
    def __init__(self,
                 port, baudrate=3000000,
                 protocol=2,
                 use_sync_read=False,
                 use_sync_write=False,
                 convert=True):
        """ At instanciation, it opens the serial port and sets the communication parameters.
            :param string port: the serial port to use (e.g. Unix (/dev/tty...), Windows (COM...)).
            :param int baudrate: default for new motors: 57600, for PyPot motors: 1000000
            :param float timeout: read timeout in seconds
            :param bool use_sync_read: whether or not to use the SYNC_READ instruction
            :param error_handler: set a handler that will receive the different errors
            :type error_handler: :py:class:`~pypot.dynamixel.error.DxlErrorHandler`
            :param bool convert: whether or not convert values to units expressed in the standard system
            :raises: :py:exc:`~pypot.dynamixel.io.DxlError` if the port is already used.
            """
        self._known_models = {}
        self._known_mode = {}

        self._sync_read = use_sync_read
        self._sync_write = use_sync_write
        self._convert = convert

        self.protocol_version = protocol
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(protocol)

        self.open_port(port, baudrate)

    def open_port(self, port, baudrate):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        self.set_port_baudrate(baudrate)
                
    def set_port_baudrate(self, baudrate):
        # Set port baudrate
        if self.portHandler.setBaudRate(baudrate):
            print("Succeeded to change the port baudrate")
        else:
            print("Failed to change the port baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def close_port(self):
        # Close port
        self.portHandler.closePort()
        
    def ping(self, motor_id):
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, motor_id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (motor_id, dxl_model_number))
            return True
        
    def scan(self, ids=range(254)):
        """ Pings all ids within the specified list, by default it finds all the motors connected to the bus. """
        return [m_id for m_id in ids if self.ping(m_id)]

    def _enable_torque(self, m_id, enable=1):
        """Enables or disables torque of a motor"""
        ADDR_PRO_TORQUE_ENABLE = 64 
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, m_id, ADDR_PRO_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            if enable: 
                print("Torque enabled for motor id {}".format(m_id))
            else: 
                print("Torque disbled for motor id {}".format(m_id))
                
    def enable_torque(self, ids):
        "list of ids as input"
        for m_id in ids:
            self._enable_torque(m_id, enable=1)
            
    def disable_torque(self, ids):
        "list of ids as input"
        for m_id in ids:
            self._enable_torque(m_id, enable=0)


    ## values for read and write are all in motor ticks
    ## conversion for units happens in corresponding set and get fucntions
            
    def write(self, ids, addr, goal):
        """ takes a list of ids and the control address of desired quantity, and the actual quantities to write to """
        if self._sync_write and len(ids)>1:
            print("Not yet implemented sync write")
        else: 
            # write something
            for i in range(len(ids)): 
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ids[i],
                                                                               addr,
                                                                               goal[i])
    def read(self, ids, addr):
        """takes a list of ids and the control address of the desired quantity wanting tobe read"""
        values = [] # list of values read
        if self._sync_read and len(ids)>1:
            print("have not implemented sync read yet")
        else: 
            # Read present something
            for m_id in ids: 
                dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, m_id,
                                                                                    addr)
                values.append(dxl_value)
                return values

    def set_goal_position(self, ids, values):
        ADDR_PRO_GOAL_POSITION = 116
        self.write(ids, ADDR_PRO_GOAL_POSITION, values)
        
    def get_goal_position(self, ids):
        ADDR_PRO_GOAL_POSITION = 116
        goal_position = self.read(ids, ADDR_PRO_GOAL_POSITION)
        return goal_position

    def get_present_position(self, ids):
        ADDR_PRO_PRESENT_POSITION = 132
        present_position = self.read(ids, ADDR_PRO_PRESENT_POSITION)
        return present_position
