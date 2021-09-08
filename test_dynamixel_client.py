import argparse
import itertools
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

way_points = [np.zeros(len(motors)), np.full(len(motors), np.pi/4)]

dxl_client = DynamixelClient(motors, device_path, baud_rate) 

for step in itertools.count():
    if step > 0 and step % 50 == 0:
        
        way_point = way_points[(step // 100) % len(way_points)]
        print('Writing: {}'.format(way_point.tolist()))
        dxl_client.write_desired_pos(motors, way_point)

    read_start = time.time()
    pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
    if step % 5 == 0:
        print('[{}] Frequency: {:.2f} Hz'.format(
            step, 1.0 / (time.time() - read_start)))
        print('> Pos: {}'.format(pos_now.tolist()))
        print('> Vel: {}'.format(vel_now.tolist()))
        print('> Cur: {}'.format(cur_now.tolist()))
            
