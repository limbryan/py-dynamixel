import numpy as np
import time
import src.io as io

from sin_controller import SinusoidController


class Hexapod():
    def __init__(self, port, ctrl_freq):

        self.port = port
        self.dxl_io = io.DxlIO(port, baudrate=2000000)
        print('Connected!')

        self.ctrl_freq = ctrl_freq
        
        self.ids = list()
        for i in range(10, 70, 10):
            for j in range(1, 4):
                self.ids.append(i + j)
        print("ids", self.ids)

        # this is always the trajectory that will be executed
        # it is a list of np.arrays the joint angles of every motor
        self._traj = []
        
    def enable_torques(self):
        self.dxl_io.enable_torque(self.ids)

    def disable_torques(self):
        self.dxl_io.disable_torque(self.ids)

    def shutdown(self):
        self.relax()
        self.dxl_io.disable_torque(self.ids)
        self.dxl_io.close_port()
        
    def relax(self):
        "Places hexapod on its belly"
        print("#### RELAX POSE CONTROLLER ####")

    def neutral_controller(self):
        "neutral pose - robot standing up with legs straight"
        print("#### NEUTRAL POSE CONTROLLER ####")
        time = 0.2
        
    def run_sin_controller(self, ctrl, time):        
        controller = SinusoidController(ctrl)
        for t in arange(0,time,self.ctrl_freq):
            command = controller.commanded_jointpos(t)
            self._traj.append(command)

        self._exec_traj()
        
    def _exec_traj():
        "execeute trajectories that are saved in _traj"
        start = time.clock()
        for i in len(self._traj):
            joint_pos = self._traj[i]
            self.dxl_io.set_goal_position(self.ids, joint_pos)
            elapsed = time.clock() - start
            time.sleep((1.0/self.ctrl_freq) - elapsed)

            if ((1.0/self.ctrl_freq) - elapsed) < 0:
                print("Control frequency is too high")

            start = time.clock()
            
        # reset the traj variable
        self._traj = []

        
def main():

    ports = io.get_available_ports()
    print('available ports:', ports)
    if not ports:
        raise IOError('No port available.')

    port = ports[0]
    print('Using the first on the list', port)


    Hexa = Hexapod(port, 30)
    
