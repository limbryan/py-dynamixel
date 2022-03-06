import argparse
import itertools
import numpy as np
import time
from dynamixel_client import DynamixelClient
import py_dynamixel.io as io

ports = io.get_available_ports()
print('available ports:', ports)
if not ports:
    raise IOError('No port available.')
port = ports[0]
print('Using the first on the list', port)
device_path = port

baud_rate = 3000000

# motor ids
motors = list()
for i in range(10, 50, 10):
    for j in range(1, 4):
        motors.append(i + j)
print("ids", motors)

#motors = [42, 43]
target = np.pi
way_points = [np.zeros(len(motors)), np.full(len(motors), target)]
targets = np.full(len(motors), target)

targets_1 = targets.copy()
targets_2 = targets.copy() + np.pi/4

#for i in range(1,12,3):
#    targets_2[i] = np.pi + np.pi/4
    
targets_list = [targets_1, targets_2]
sel=1

print("Motors: ", motors)
print("Device: ", device_path)
print("Baudrate: ", baud_rate)

ctrl_freq = 50. # Hz
ctrl_dt = 1./ctrl_freq

start = time.time()

#dxl_client = DynamixelClient(motors, device_path, baud_rate, lazy_connect=True) 
with DynamixelClient(motors, device_path, baud_rate, lazy_connect=True) as dxl_client:

    while(1):

        print("Targets: ", targets_list[sel])
        
        dxl_client.write_desired_pos(motors, targets_list[sel])
        
        elapsed = time.time() - start
        print(int(elapsed))
        #pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
        
        if int(elapsed) % 5 == 4:
            print("CHANGE TARGET")
            if sel == 0:
                sel = 1
            else:
                sel = 0
    
    

    
    

'''
for step in itertools.count():
    write_start = time.time()
    if step > 0 and step % 50 == 0:
        way_point = way_points[(step // 100) % len(way_points)]

        print('Writing: {}'.format(way_point.tolist()))
        dxl_client.write_desired_pos(motors, way_point)

        #print('[{}] Write Frequency: {:.2f} Hz'.format(
        #    step, 1.0 / (time.time() - write_start)))

    
    read_start = time.time()
    pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
    if step % 50 == 0:
        print('[{}] Read Frequency: {:.2f} Hz'.format(
            step, 1.0 / (time.time() - read_start)))
        print('> Position: {}'.format(pos_now.tolist()))
        print('> Velocity: {}'.format(vel_now.tolist()))
        print('> Current: {}'.format(cur_now.tolist()))

        if pos_now.tolist() == target:
            way_points = [np.zeros(len(motors)), np.full(len(motors), -target)]

    '''
