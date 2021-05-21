import argparse
import sys

import matplotlib.pyplot as plt
import pandas as pd

import src.io as io
from hexapod_controller import Hexapod

## Example to task Hexapod class ##
ports = io.get_available_ports()
print('available ports:', ports)
if not ports:
    raise IOError('No port available.')

port = ports[0]
print('Using the first on the list', port)

ctrl_freq = 100
Hexa = Hexapod(port, ctrl_freq)
Hexa.neutral_controller()


def key_event(event, args):
    if event.key == 'escape':
        Hexa.shutdown()
        sys.exit(0)


def click_event(event, args):
    '''
    # reutrns a list of tupples of x-y points
    click_in = plt.ginput(1,-1) # one click only, should not timeout
    print("click_in: ",click_in)
    
    selected_cell = [int(click_in[0][0]), int(click_in[0][1])]
    print(selected_cell)
    selected_x = selected_cell[0]
    selected_y = selected_cell[1]
    '''
    # event.button ==1 is the left mouse click
    if event.button == 1:
        selected_x = int(event.xdata)
        selected_y = int(event.ydata)
        selected_solution = data[(data["x_bin"] == selected_x) & (data["y_bin"] == selected_y)]
        

        # For hexapod omnitask
        print("SELECTED SOLUTION SHAPE: ", selected_solution.shape)
        selected_solution = selected_solution.iloc[0, :]
        selected_ctrl = selected_solution.iloc[5:-4].to_numpy() # bryan archive
        #selected_ctrl = selected_solution.iloc[4:-4].to_numpy() # luca archive
        print("Selected ctrl shape: ", selected_ctrl.shape)
        # print(selected_ctrl[0].shape) #(1,36)

        # hexapod uni
        # selected_solution = selected_solution.iloc[0, :]
        # selected_ctrl = selected_solution.iloc[8:-2].to_numpy()

        # print("Selected ctrl shape: ", selected_ctrl.shape) # should be 3661
        # print("Selected descriptor bin: " ,selected_x, selected_y)
        print("Selected descriptor from archive: ", selected_solution.iloc[1:3].to_numpy())
        # print("Selected fitness from archive: ", selected_solution.iloc[0])

        # ---- PLAY THE SELECTED CONTROLLER -----#        
        Hexa.run_sin_controller(selected_ctrl, duration=3.0)
        Hexa.neutral_controller()
        print("EXECUTE CONTROLLER")


def read_archive_luca(filename):
    data = pd.read_csv(filename, delim_whitespace=True)
    # data = data.iloc[:,:-1] # drop the last column which was made because there is a comma after last value i a line
    # data = np.loadtxt(args.filename)

    # exchanging x & y axis + inverting left and right.
    data["scale_x"] = (-1 * data.iloc[:, 2] + 1.2) / 2.4
    data["scale_y"] = (data.iloc[:, 1] + 1.2) / 2.4

    # For Hexapod
    data['x_bin'] = pd.cut(x=data["scale_x"],
                           bins=[p / 100 for p in range(101)],
                           labels=[p for p in range(100)])
    data['y_bin'] = pd.cut(x=data["scale_y"],
                           bins=[p / 100 for p in range(101)],
                           labels=[p for p in range(100)])

    return data

def read_archive_bryan(filename):
    data = pd.read_csv(filename)
    data = data.iloc[:,:-1] # drop the last column which was made because there is a comma after last value i a line

    # exchanging x & y axis + inverting left and right.
    data["scale_x"] = data.iloc[:, 1]
    data["scale_y"] = data.iloc[:, 2] 
    
    # For Hexapod
    data['x_bin'] = pd.cut(x=data["scale_x"],
                           bins=[p / 100 for p in range(101)],
                           labels=[p for p in range(100)])
    data['y_bin'] = pd.cut(x=data["scale_y"],
                           bins=[p / 100 for p in range(101)],
                           labels=[p for p in range(100)])

    return data




if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", type=str)  # file to visualize rollouts from
    parser.add_argument("--plot_type", type=str, default="grid", help="scatter plot or grid plot")

    args = parser.parse_args()

    #data = read_archive_luca(args.filename)
    data = read_archive_bryan(args.filename)

    # cmap = matplotlib.cm.get_cmap('Spectral') # Getting a list of color values
    # data['color_dict'] = pd.Series({k:cmap(1) for k in data['scaled_x']})

    # =====================PLOT DATA===========================#

    # FOR BINS / GRID
    if args.plot_type == "grid":
        fig, ax = plt.subplots()
        # data.plot.scatter(x="x_bin", y="y_bin", c=0, colormap="viridis", s=2, ax=ax) # bryan archive
        data.plot.scatter(x="x_bin", y="y_bin", c=3, colormap="viridis", s=2, ax=ax) # luca archive

        plt.xlim(0, 100)
        plt.ylim(0, 100)
    else:
        # fig, ax = plt.subplots(nrows=1, ncols=2)
        fig, ax = plt.subplots()

        # FOR JUST A SCATTER PLOT OF THE DESCRIPTORS - doesnt work for interactive selection
        data.plot.scatter(x=2,y=3,c=0,colormap='Spectral', s=2, ax=ax, vmin=-0.1, vmax=1.2)
        #data.plot.scatter(x=1, y=2, c=0, colormap='viridis', s=2, ax=ax)

        # data.plot.scatter(x=1,y=2,s=2, ax=ax[0])
        # data.plot.scatter(x=3,y=4,c=0,colormap='viridis', s=2, ax=ax)
        # data.plot.scatter(x=4,y=5,s=2, ax=ax[1])
        # plt.xlim(-0.5,0.5)
        # plt.ylim(-0.5,0.5)

    # event to look out. visualization or closing the plot
    fig.canvas.mpl_connect('key_press_event', lambda event: key_event(event, args))
    fig.canvas.mpl_connect('button_press_event', lambda event: click_event(event, args))

    plt.show()
