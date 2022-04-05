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
        
        for t, tim in enumerate(np.arange(0,duration,1.0/self.ctrl_freq)):
            # get control parameters
            
            amplitudes_top = ctrl[np.asarray([0, 4, 8, 12])]
            phases_top = ctrl[np.asarray([1, 5, 9, 13])]
            amplitudes_bottom = ctrl[np.asarray([2, 6, 10, 14])]
            phases_bottom = ctrl[np.asarray([3, 7, 11, 15])]

            # get joint positions from the sin controller based on parameters - given timestep
            top_actions = self.simple_cosine_controller(amplitudes_top, phases_top, t)
            bottom_actions = self.simple_sine_controller(amplitudes_bottom, phases_bottom, t)

            actions = np.zeros(12)
            actions[np.asarray([0, 3, 6, 9])] = -top_actions
            actions[np.asarray([1, 4, 7, 10])] = -bottom_actions
            actions[np.asarray([2, 5, 8, 11])] = -bottom_actions #-1 *bottom_actions
            #print(actions)
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

    ctrl_freq = 100
    Hexa = Quadruped(port, ctrl_freq)

    # TRIPOD GAIT
    #ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5,
    #        1, 0.5, 0.5, 0.25, 0.75, 0.5,
    #        1, 0, 0.5, 0.25, 0.25, 0.5,
    #        1, 0, 0.5, 0.25, 0.75, 0.5,]

    #ctrl = [-0.7840116, -0.13674164, -0.09958146, -0.256268, -0.8670167, 0.18974361,
    #        0.1044462, -0.12208096, 0.76759064, -0.04266164, 0.44489166, -0.02280497,
    #        0.8566111, -0.09534891, 0.6469337, -0.62066144]
    #ctrl = [-0.96937984, -0.36611515,  0.07884257,  0.27127394,
    #         -0.9935326 , -0.4245077 ,  0.08549317,  0.5454599 ,
    #          0.8169315 ,  0.30566746, -0.8654227 , -0.548556  ,
    #         -0.86666745,  0.68196094,  0.39499295,  0.7395957 ]

    ctrl = [-0.02151023, -0.04896656,  0.0726631 ,  0.10821316, # leg 1
              0.10432949,  0.00208104,  0.12384317, -0.0074433 , # leg 2
             -0.83339214,  0.08297074, -0.27983293, -0.09963401, # leg 3
              0.79204786, -0.12445881,  0.22914657, -0.01037501] # leg 4

    ctrl = [-0.02151023, -0.04896656,  0.0726631 ,  0.10821316, # leg 1
            0.79204786, -0.12445881,  0.22914657, -0.01037501, # leg 4
            0.10432949,  0.00208104,  0.12384317, -0.0074433 , # leg 2
            -0.83339214,  0.08297074, -0.27983293, -0.09963401] # leg 3
    ctrl = [ 0.95487738, -0.19840202, -0.37690488, -0.90610719, -0.70346367,
             -0.83066481, -0.41591096, -0.81542987,  0.98675978, -0.13365969,
             0.38640368,  0.90178722, -0.95061338,  0.80209345, -0.75718516,
             -0.18783724]
    ctrl = [ 0.91339844, -0.03445202, -0.24464227, -0.96043158, -0.96875203,
             -0.96279687, -0.12827234, -0.75980312,  0.98761266, -0.51050717,
             0.08923282,  0.78591275, -0.94271386,  0.8408103 , -0.56126618,
             -0.31806946]
    ctrl = [-0.9940834 , -0.89154595, -0.9863003 ,  0.09467936,
              0.9612475 ,  0.27205577, -0.5316649 ,  0.3537691 ,
             -0.9788035 , -0.9143969 , -0.9327545 , -0.36428037,
             -0.86179745,  0.8935081 ,  0.8693234 , -0.9974186 ]
    ctrl = [-0.9764429 , -0.25240344,  0.568361  , -0.5238106 ,
              0.25995094,  0.8380455 , -0.77424544, -0.22890283,
              0.66352206,  0.84589976, -0.8760021 , -0.47111103,
              0.9171952 , -0.85577905, -0.58049345, -0.40971956]
    ctrl = [ 0.8797216 , -0.76216435, -0.13483346,  0.83845174,
             -0.48399383, -0.9820581 ,  0.8029403 ,  0.93468094,
             -0.5836343 ,  0.59626836,  0.8907593 ,  0.68705434,
             -0.84334415, -0.8765186 , -0.20524895, -0.87705386]


    ctrl = [ 0.79472595, -0.64195836, -0.045003  ,  0.78829575,
             -0.807188  ,  0.7082003 ,  0.5546785 , -0.23643558,
              0.94696194,  0.3079133 ,  0.95564395,  0.67207706,
              0.8078228 ,  0.76797193,  0.9327098 , -0.81448823]

    #ctrl = [1.0, 0.0, 0.5, 0.25,
    #        -1.0, 0.0, 0.5, 0.25,
    #        1.0, 0, 0.5, -0.25,
    #        -1.0, 0, 0.5, -0.25]
    '''
    ctrl = [-0.16789281, -0.6912696 , -0.08312657, 0.917392 ,
            0.7344484 , 0.53297126, 0.5989111 , 0.09726007,
            0.80249286, -0.8352062 , 0.75022465, 0.6299669 ,
            0.72547114, -0.8218682 , -0.73964155, 0.5545298 ]

    ctrl = [ 0.92880154, -0.9310799 , 0.4899586 , -0.04429985,
             -0.63943887, -0.80240124, 0.40971965, 0.9301228 ,
             0.7423949 , -0.47411442, 0.3960832 , 0.9621112 ,
             0.3312904 , 0.83120865, 0.02415982, -0.76639307]
    ctrl = ctrl

    '''
    ctrl = [ 0.09751841, -0.71711195, -0.1021072 , 0.87681025,
             -0.804326 , -0.9010009 , -0.06168101, -0.8300243 ,
             -0.93022394, -0.8638982 , -0.91489714, 0.7070501 ,
             -0.29102728, 0.07217796, 0.66004676, 0.7918089 ]
    ctrl = [-0.32297555, -0.8778983 , 0.9392574 , 0.8149893 , 0.6556681 , -0.2730729 , -0.92238206, 0.26854864, 0.8856505 , 0.8181462 , 0.29273996, 0.93530816, 0.8177438 , 0.37596726, -0.85127324, -0.8839844 ]

    ctrl = [-0.7145506 , 0.5882103 , -0.49909553, 0.04267859, 0.46944514, 0.7952452 , -0.8241356 , 0.39554325, 0.38773456, -0.32591304, 0.28372228, 0.91598004, -0.36659238, 0.6883743 , -0.7370194 , -0.20900308]

    ctrl = [ 0.36843976, 0.18968934, 0.23121159, 0.25038326, -0.62300575, 0.07276913, 0.08777036, 0.04851788, -0.33288538, 0.5488871 , -0.6025577 , -0.3496894 , 0.11855289, -0.07134574, -0.03362073, 0.37899032]

    ctrl = [-0.9516594 , 0.5029124 , -0.8863376 , -0.9122829 , 0.97301924, 0.658295 , -0.81284696, 0.80638856, -0.67644304, -0.24345534, 0.97683215, 0.8781318 , -0.02782184, 0.82009876, -0.91428846, 0.3673589 ]

    ctrl = [ 0.9400246 , -0.5648328 , 0.6486923 , 0.7421423 , 0.05369389, -0.4147584 , 0.81264776, -0.37701032, -0.2149623 , -0.38221115, 0.70301527, 0.07598443, -0.8033788 , 0.32907248, 0.29239598, 0.42540807]

    ctrl = [-0.6191339 , -0.00869758, 0.08034393, -0.34647357, -0.7816453 , 0.8126544 , -0.02539819, -0.03454808, 0.72835165, 0.2531592 , -0.10565615, 0.12811454, -0.5659985 , 0.00621947, -0.728982 , -0.3037064 ]
    Hexa.neutral_controller()
    #Hexa.relax()
    #Hexa.run_sin_controller(ctrl, duration=3.0)
    Hexa.run_new_sin_controller(ctrl, duration=4.0)
    Hexa.shutdown()
    

if __name__ == "__main__":
    main()
