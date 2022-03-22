import os, sys
import glob
import numpy as np
from dynamixel_sdk import *
import py_dynamixel.conversion as conv

### MOTOR ADDRESSES - 
## Addresses for XM430-W350
#ADDR_PRO_TORQUE_ENABLE = 64
#ADDR_PRO_LED = 65
#ADDR_PRO_POSITION_D = 80
#ADDR_PRO_POSITION_I = 82
#ADDR_PRO_POSITION_P = 84
#ADDR_PRO_GOAL_POSITION = 116
#ADDR_PRO_PRESENT_CURRENT = 126
#ADDR_PRO_PRESENT_VELOCITY = 128
#ADDR_PRO_PRESENT_POSITION = 132

# Data Byte Length                                                                                                                                                    
#LEN_PRO_LED             = 1
#LEN_PRO_GOAL_POSITION       = 4
#LEN_PRO_PRESENT_POSITION    = 4
#LEN_PRO_PRESENT_VELOCITY    = 4

## Addresses for XL-321
ADDR_PRO_TORQUE_ENABLE = 24
ADDR_PRO_LED = 25
ADDR_PRO_POSITION_D = 27
ADDR_PRO_POSITION_I = 28
ADDR_PRO_POSITION_P = 29
ADDR_PRO_GOAL_POSITION = 30
ADDR_PRO_PRESENT_CURRENT = 45
ADDR_PRO_PRESENT_VELOCITY = 39
ADDR_PRO_PRESENT_POSITION = 37

# Data Byte Length                                                                                                                                                
LEN_PRO_LED             = 1
LEN_PRO_GOAL_POSITION       = 2
LEN_PRO_PRESENT_POSITION    = 2
LEN_PRO_PRESENT_VELOCITY    = 2



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
                 use_bulk_read=False,
                 use_bulk_write=False,
                 convert=True,
                 use_sync_read=False,
                 use_sync_write=False):
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
        
        self.protocol_version = protocol
        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(protocol)

        self._sync_write = use_sync_write
        self._sync_read = use_sync_read

        self._bulk_read = use_bulk_read
        self._bulk_write = use_bulk_write

        # Bulk definition
        if use_bulk_read: 
            # Initialize GroupBulkWrite instance                                                                                                            
            self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)
        if use_bulk_write:
            # Initialize GroupBulkRead instace for Present Position                                                                                  
            self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

        # Sync definitions
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
        self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        self.groupSyncReadVelocity = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY)

        self._convert = convert
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

    def configure(self, ids):
        ADDR_PRO_RETURN_LEVEL = 17
        RETURN_LEVEL = 1
        ADDR_PRO_DELAY_TIME = 5
        DELAY = 0
        self.portHandler.setPacketTimeoutMillis(1)
        for id in ids: 
            self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_PRO_RETURN_LEVEL, RETURN_LEVEL)
            self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_PRO_DELAY_TIME, DELAY)
        
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
    
    def broadcast_ping(self):
        # Try to broadcast ping the Dynamixel                                                                                      
        dxl_data_list, dxl_comm_result = self.packetHandler.broadcastPing(self.portHandler)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        found_ids = []
        print("Detected Dynamixel :")
        for dxl_id in dxl_data_list:
            print("[ID:%03d] model version : %d | firmware version : %d" % (dxl_id, dxl_data_list.get(dxl_id)[0], dxl_data_list.get(dxl_id)[1]))
            found_ids.append(dxl_id)
            
        return found_ids
    
    def scan(self, ids=range(254)):
        """ Pings all ids within the specified list, by default it finds all the motors connected to the bus. """
        #found_ids = [m_id for m_id in ids if self.ping(m_id)] # if using for loop over ids
        found_ids = self.broadcast_ping() # if using broadcast ping - faster
        return found_ids

    def _enable_torque(self, m_id, enable=1):
        """Enables or disables torque of a motor"""
        
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


    def _get_byte_array_sync(self, dxl_goal_position_pulse):
        return[DXL_LOBYTE(dxl_goal_position_pulse), DXL_HIBYTE(dxl_goal_position_pulse)
        ]
        #return [
        #    DXL_LOBYTE(DXL_LOWORD(dxl_goal_position_pulse)),
        #    DXL_HIBYTE(DXL_LOWORD(dxl_goal_position_pulse)),
        #    DXL_LOBYTE(DXL_HIWORD(dxl_goal_position_pulse)),
        #    DXL_HIBYTE(DXL_HIWORD(dxl_goal_position_pulse))
        #]

    def init_bulk_read(self, list_ids):
        print("INIT BULK READ")
        for dxl_id in list_ids:
            # Add parameter storage for all Dynamixel ids present position
            dxl_addparam_result = self.groupBulkRead.addParam(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % dxl_id)
                quit()
            dxl_addparam_result = self.groupBulkRead.addParam(dxl_id, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id)
    
        return 1
    
    def init_bulk_write(self): 

        return 0

    def init_sync_read(self, list_ids):
        print("INIT SYNC READ")
        for dxl_id in list_ids:
            dxl_addparam_result = self.groupSyncReadPosition.addParam(dxl_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
            dxl_addparam_result = self.groupSyncReadVelocity.addParam(dxl_id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
        return 1
    
    ## values for read and write are all in motor ticks
    ## conversion for units happens in corresponding set and get fucntions
    def write(self, list_ids, addr, array_goals_pulses):
        """ takes a list of ids and the control address of desired quantity, and the actual quantities to write to """
        if self._sync_write and len(list_ids)>1:
            for dxl_id, goal_position_pulse in zip(list_ids, array_goals_pulses):
                dxl_addparam_result = self.groupSyncWrite.addParam(dxl_id, self._get_byte_array_sync(goal_position_pulse))
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
                    # quit()

            dxl_comm_result = self.groupSyncWrite.txPacket()

            self.groupSyncWrite.clearParam()
        else:
            # write something
            # write the correct amount of bytes depending on the robot - if xl-320 write2byte, if xm-430 write4byte
            for i in range(len(list_ids)):
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, list_ids[i],
                                                                               addr,
                                                                               array_goals_pulses[i])
                print(dxl_comm_result, dxl_error)
            
                
    def read(self, list_ids, addr, quantity=None):
        """takes a list of ids and the control address of the desired quantity wanting tobe read"""
        values = [] # list of values read
        
        if self._sync_read and len(list_ids)>1 and (quantity is not None):
            if quantity == "pos":
                dxl_comm_result = self.groupSyncReadPosition.txRxPacket()
                #if dxl_comm_result != COMM_SUCCESS:
                #    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                for dxl_id in list_ids:
                    #dxl_getdata_result = self.groupSyncReadPosition.isAvailable(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                    #if dxl_getdata_result != True:
                    #    print("[ID:%03d] groupSyncReadPos getdata failed" % dxl_id)
                        #quit()
                    dxl_pres_result = self.groupSyncReadPosition.getData(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)  
                    values.append(dxl_pres_result)

            elif quantity == "vel":
                dxl_comm_result = self.groupSyncReadVelocity.txRxPacket()
                #if dxl_comm_result != COMM_SUCCESS:
                    #print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                for dxl_id in list_ids:
                    #dxl_getdata_result = self.groupSyncReadVelocity.isAvailable(dxl_id, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY)
                    #if dxl_getdata_result != True:
                    #    print("[ID:%03d] groupSyncReadVel getdata failed" % dxl_id)
                        #quit()
                    dxl_pres_result = self.groupSyncReadVelocity.getData(dxl_id, ADDR_PRO_PRESENT_VELOCITY, LEN_PRO_PRESENT_VELOCITY)  
                    values.append(dxl_pres_result)
                                        
        else: 
            # Read present something
            for m_id in list_ids: 
                dxl_value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, m_id, addr)
                values.append(dxl_value)

        values = np.array(values)
        return values

    def set_goal_position(self, ids, values, units="rads", motor_type='xl'):
        if units == "rads":
            values = conv.rads_to_pulses(values, motor_type=motor_type)
        elif units == "deg":
            values = conv.degree_to_pulses(values, motor_type=motor_type)
        elif units == "pulses":
            values = values
        
        self.write(ids, ADDR_PRO_GOAL_POSITION, values)
        
    def get_goal_position(self, ids):
        goal_position = self.read(ids, ADDR_PRO_GOAL_POSITION)
        goal_position = conv.pulses_to_degree(goal_position)
        return goal_position

    def get_present_position(self, ids, units="rads"):     
        present_position = self.read(ids, ADDR_PRO_PRESENT_POSITION, "pos")
        if units == "rads":
            present_position = conv.pulses_to_rads(present_position)
        elif units == "deg":
            present_position = conv.pulses_to_degree(present_position)
            
        return present_position

    def get_present_velocity(self, ids):     
        present_velocity = self.read(ids, ADDR_PRO_PRESENT_VELOCITY, "vel")
        present_velocity = conv.rpm_to_radps(present_velocity)
        return present_velocity

    def get_present_current(self, ids):     
        present_current = self.read(ids, ADDR_PRO_PRESENT_CURRENT)
        return present_current

