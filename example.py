import numpy as np
import time
import src.io as io

ports = io.get_available_ports()
print('available ports:', ports)
if not ports:
    raise IOError('No port available.')

port = ports[0]
print('Using the first on the list', port)

dxl_io = io.DxlIO(port, baudrate=3000000)
print('Connected!')

found_ids = dxl_io.scan(range(10))
print('Found ids:', found_ids)


ids = found_ids

dxl_io.enable_torque(ids)

dxl_io.set_goal_position(ids, np.array([0, 90]))
time.sleep(5)
cur_pos = dxl_io.get_present_position(ids)
print("Current posiiton: ", cur_pos)

dxl_io.set_goal_position(ids, np.array([180, 0]))
time.sleep(5)
cur_pos = dxl_io.get_present_position(ids)
print("Current posiiton: ", cur_pos)

dxl_io.disable_torque(ids)



dxl_io.close_port()
