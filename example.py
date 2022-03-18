import numpy as np
import time
import py_dynamixel.io as io

ports = io.get_available_ports()
print('available ports:', ports)
if not ports:
    raise IOError('No port available.')

port = ports[0]
print('Using the first on the list', port)

dxl_io = io.DxlIO(port, baudrate=1000000, use_sync_write=False, use_sync_read=False)
print('Connected!')

#ids = list()
#for i in range(10, 70, 10):
#    for j in range(1, 4):
#        ids.append(i + j)
#print("ids", ids)

# found_ids = set()
# while len(found_ids) < 18:
#     new_ids = dxl_io.scan(range(200))
#     found_ids = set(new_ids).union(found_ids)
#     print('Found ids:', found_ids)

ids = dxl_io.scan(range(200))
print(ids) 
#ids = [2,3]
#print(ids)
dxl_io.configure(ids)
#dxl_io.init_sync_read(ids)
#pulse_commands = np.ones(12, dtype=int)*512
degree_commands = np.ones(12)*np.pi/4
dxl_io.enable_torque(ids)
dxl_io.set_goal_position(ids, degree_commands, units="rads")
time.sleep(5)
#dxl_io.set_goal_position(ids, np.array([45, 45]), units="deg")


'''
dxl_io.set_goal_position([2, 3], np.array([ANGLE_1, ANGLE_1]), units="deg")
time.sleep(2)
dxl_io.set_goal_position([2, 3], np.array([ANGLE_2, ANGLE_2]), units="deg")
time.sleep(2)
dxl_io.set_goal_position([2, 3], np.array([ANGLE_1, ANGLE_1]), units="deg")
time.sleep(2)
dxl_io.set_goal_position([2, 3], np.array([ANGLE_2, ANGLE_2]), units="deg")
time.sleep(2)
dxl_io.set_goal_position([2, 3], np.array([ANGLE_1, ANGLE_1]), units="deg")
time.sleep(2)
dxl_io.set_goal_position([2, 3], np.array([ANGLE_2, ANGLE_2]), units="deg")
'''
#time.sleep(2)

# dxl_io.set_goal_position(ids, np.array([180, 220, 220] * 6), units="deg")
# time.sleep(1.5)
# dxl_io.set_goal_position(ids, np.array([180, 180, 180] * 6), units="deg")
# time.sleep(1.5)
# dxl_io.set_goal_position(ids, np.array([180, 220, 220] * 6), units="deg")
# time.sleep(1.5)
# dxl_io.set_goal_position(ids, np.array([180, 180, 180] * 6), units="deg")
# time.sleep(1.5)
# dxl_io.set_goal_position(ids, np.array([180, 220, 220] * 6), units="deg")
# time.sleep(1.5)
# dxl_io.set_goal_position(ids, np.array([180, 180, 180] * 6), units="deg")
# time.sleep(1.5)

#while (1): 
#    cur_pos = dxl_io.get_present_position(ids)
#    cur_vel = dxl_io.get_present_velocity(ids)
#    if not np.all(cur_pos == 0):
#        print("Current posiiton: ", cur_pos)
#        print("Current velocity: ", cur_vel)
    #time.sleep(1)

# # dxl_io.set_goal_position(ids, np.array([180, 0]))
# time.sleep(5)
# cur_pos = dxl_io.get_present_position(ids)
# print("Current posiiton: ", cur_pos)

dxl_io.disable_torque(ids)
dxl_io.close_port()

