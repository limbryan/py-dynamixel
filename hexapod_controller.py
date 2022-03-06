import numpy as np
import time
import py_dynamixel.io as io

from sin_controller import SinusoidController

class Hexapod():
    def __init__(self, port, ctrl_freq):

        self.port = port
        self.dxl_io = io.DxlIO(port, baudrate=2000000, use_sync_write=True, use_sync_read=False)
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

        self.enable_torques()
        self.dxl_io.init_sync_read(self.ids)

        
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
        duration = 2.0
        for t in np.arange(0,duration,1.0/self.ctrl_freq):
            command = np.array([np.pi, np.pi, np.pi]*6)
            # second joint values have to follow a trajectory
            a = (duration-t)/duration
            b = t/duration
            for i in range(1,18,3):
                command[i] += (a*(np.pi/4) / 6.0 + b*((np.pi/2)*1.2))
            
            self._traj.append(command)

        self._exec_traj()
            
    def neutral_controller(self):
        "neutral pose - robot standing up with legs straight"
        print("#### NEUTRAL POSE CONTROLLER ####")
        duration = 0.1
        for t in np.arange(0,duration,1.0/self.ctrl_freq):
            command = np.array([np.pi, np.pi, np.pi]*6)
            self._traj.append(command)

        self._exec_traj()
        time.sleep(0.5)
        
    def run_sin_controller(self, ctrl, duration):        
        controller = SinusoidController(ctrl)
        for t in np.arange(0,duration,1.0/self.ctrl_freq):
            command = controller.commanded_jointpos(t)
            command = command+np.pi # offset differnce from simulator to real world configuration
            self._traj.append(command)

        self._exec_traj()
        time.sleep(0.5)
        
    def _exec_traj(self):
        "execeute trajectories that are saved in _traj"
        start = time.time()
        for i in range(len(self._traj)):
            # get current state
            #cur_jpos = self.dxl_io.get_present_position(self.ids)
            #cur_jvel = self.dxl_io.get_present_velocity(self.ids)
            #print(cur_jpos, cur_jvel)

            # get action
            joint_pos = self._traj[i]
            self.dxl_io.set_goal_position(self.ids, joint_pos)
            elapsed = time.time() - start
            time.sleep((1.0/self.ctrl_freq) - elapsed)
            
            if ((1.0/self.ctrl_freq) - elapsed) < 0:
                print("Control frequency is too high")

            start = time.time()

        # reset the traj variable
        self._traj = []

    # add some functions for conversion to hexapod default angles - i.e neutral position is 180 deg absolute
    # especially sin controller commands

    
def main():
    ## Example to task Hexapod class ##
    ports = io.get_available_ports()
    print('available ports:', ports)
    if not ports:
        raise IOError('No port available.')

    port = ports[0]
    print('Using the first on the list', port)

    ctrl_freq = 100
    Hexa = Hexapod(port, ctrl_freq)

    # TRIPOD GAIT
    ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5,
            1, 0.5, 0.5, 0.25, 0.75, 0.5,
            1, 0, 0.5, 0.25, 0.25, 0.5,
            1, 0, 0.5, 0.25, 0.75, 0.5,
            1, 0.5, 0.5, 0.25, 0.25, 0.5,
            1, 0, 0.5, 0.25, 0.75, 0.5]
    ctrl = np.array(ctrl)

    Hexa.neutral_controller()
    for i in range(100):
        time.sleep(1)

    #Hexa.relax()
    #Hexa.run_sin_controller(ctrl, duration=2.0)
    Hexa.shutdown() # shutdown already contains relax
    

if __name__ == "__main__":
    main()
