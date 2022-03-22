import numpy as np
import time
import py_dynamixel.io as io

from sin_controller import SinusoidController

class Quadruped():
    def __init__(self, port, ctrl_freq):

        self.port = port
        self.dxl_io = io.DxlIO(port, baudrate=1000000, use_sync_write=True, use_sync_read=True)
        print('Connected!')

        self.ctrl_freq = ctrl_freq
        
        self.ids = list()
        for i in range(10, 50, 10):
            for j in range(1, 4):
                self.ids.append(i + j)
        print("ids", self.ids)

        # this is always the trajectory that will be executed
        # it is a list of np.arrays the joint angles of every motor
        self._traj = []

        self.dxl_io.configure(self.ids)
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
            command = np.array([0.0, 0.0, 0.0]*4)
            # second joint values have to follow a trajectory
            a = (duration-t)/duration
            b = t/duration
            for i in range(1,12,3):
                command[i] += (a*(np.pi/4) / 6.0 + b*((np.pi/2)*1.2))
            
            self._traj.append(command)

        self._exec_traj()
            
    def neutral_controller(self):
        "neutral pose - robot standing up with legs straight"
        print("#### NEUTRAL POSE CONTROLLER ####")
        duration = 0.1
        for t in np.arange(0,duration,1.0/self.ctrl_freq):
            command = np.array([0, 0, 0]*4)
            self._traj.append(command)

        self._exec_traj()
        time.sleep(0.5)
        
    def run_sin_controller(self, ctrl, duration):        
        controller = SinusoidController(ctrl)
        for t in np.arange(0,duration,1.0/self.ctrl_freq):
            command = controller.commanded_jointpos(t)
            command = command # offset differnce from simulator to real world configuration
            self._traj.append(command)

        self._exec_traj()
        time.sleep(0.5)

    def simple_sine_controller(self,amplitude, phase, t):
        return (np.pi / 3 *amplitude * np.sin((t*np.pi/25)+phase))# * 180 / np.pi
    
    def simple_cosine_controller(self,amplitude,phase,t):
        return (np.pi / 4 * amplitude * np.cos((t*np.pi/25)+phase))# * 180 / np.pi

        
    def run_new_sin_controller(self, ctrl, duration):

        ctrl = np.asarray(ctrl)
        for t in np.arange(0,duration,1.0/self.ctrl_freq):
            # get control parameters
            amplitudes_top = ctrl[np.aarray([0, 4, 8, 12])]
            phases_top = ctrl[np.asarray([1, 5, 9, 13])]
            amplitudes_bottom = ctrl[np.asarray([2, 6, 10, 14])]
            phases_bottom = ctrl[np.asarray([3, 7, 11, 15])]

            # get joint positions from the sin controller based on parameters - given timestep
            top_actions = self.simple_cosine_controller(amplitudes_top, phases_top, t)
            bottom_actions = self.simple_sine_controller(amplitudes_bottom, phases_bottom, t)

            actions = np.zeros(12)
            actions[np.asarray([0, 3, 6, 9])] = top_actions
            actions[np.asarray([1, 4, 7, 10])] = bottom_actions
            actions[np.asarray([2, 5, 8, 11])] = -1 * bottom_actions

            command = actions
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

    ctrl_freq = 60
    Hexa = Quadruped(port, ctrl_freq)

    # TRIPOD GAIT
    #ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5,
    #        1, 0.5, 0.5, 0.25, 0.75, 0.5,
    #        1, 0, 0.5, 0.25, 0.25, 0.5,
    #        1, 0, 0.5, 0.25, 0.75, 0.5,]

    #ctrl = [-0.7840116, -0.13674164, -0.09958146, -0.256268, -0.8670167, 0.18974361,
    #        0.1044462, -0.12208096, 0.76759064 -0.04266164, 0.44489166, -0.02280497
    #        0.8566111, -0.09534891, 0.6469337, -0.62066144]
    #ctrl = np.array(ctrl)

    Hexa.neutral_controller()
    #Hexa.relax()
    #Hexa.run_sin_controller(ctrl, duration=3.0)
    #Hexa.run_new_sin_controller(ctrl, duration=3.0)
    Hexa.shutdown()
    

if __name__ == "__main__":
    main()
