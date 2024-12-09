import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.colors import Normalize
import time

def read_traj(data_file):
    # Define lists to store data
    x = []
    y = []
    z = []
    
    # Read data from the file
    with open(data_file, "r") as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            x.append(float(state[0]))
            y.append(float(state[1]))
            z.append(float(state[2]))
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    return x,y,z

def read_vel(data_file):
    # Define lists to store data
    vx = []
    vy = []
    vz = []
    
    # Read data from the file
    with open(data_file, "r") as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            vx.append(float(state[3]))
            vy.append(float(state[4]))
            vz.append(float(state[5]))
    vx = np.array(vx)
    vy = np.array(vy)
    vz = np.array(vz)
    return vx, vy, vz

def read_U(data_file):
    # Define lists to store data
    x = []
    y = []
    
    # Read data from the file
    with open(data_file, "r") as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            x.append(float(state[0]))
            y.append(float(state[1]))
    x = np.array(x)
    y = np.array(y)
    return x,y

def get_radius_distance(X, Y, Z):
    d = []
    for i in range(len(X)):
        x = abs(X[i])
        y = abs(Y[i])
        z = abs(Z[i])
        d.append(np.sqrt(x*x+y*y+z*z))
    d = np.array(d)
    return d

def plot_traj_ompl(data_file):
    data = np.loadtxt(data_file)

    # Create a 3D plot
    ax = generate3Dplot()

    # Plot the 3D line
    ax.plot(data[:,0], data[:,1], data[:,2], c='black', label='Trajectory')
    ax.scatter(data[0,0], data[0,1], data[0,2], c='black', label='x0')
    ax.scatter(data[-1,0], data[-1,1], data[-1,2], c='red', label='xf')

    # Add a legend
    ax.legend()
    # Show the 3D plot
    ax.set_aspect('equal')
    plt.show()

def create_plots(file_traj):
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Add 3D scatter points
    ax.scatter(0, 0, 0, s=100, c='g', marker='o', label='Asteroid')  

    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_aspect('equal')

    # Plot graph
    plot_graph("/Users/chko1829/src/SolarSailLanding/file_dump/graph.csv", ax)

    # Plot path
    plot_path("/Users/chko1829/src/SolarSailLanding/file_dump/path.csv", ax, "-", "red", "waypoints")

    # Plot traj
    #plot_traj(file_traj, ax, ":", "r", "traj", isCool=True)

    # Add a legend
    ax.legend()

    # Show the 3D plot
    plt.show()

def plot_graph(data_file, ax):

    # Initialize a row count variable
    row_count = 0
    # Open the CSV file and count the rows
    with open(data_file, 'r') as file:
        for line in file:
            row_count += 1
    max_edge = 400
    skip_factor = 1
    if(row_count > max_edge):
        skip_factor = row_count//max_edge

    # Read data from the file
    with open(data_file, "r") as file:
        lines = file.readlines()
        i = 0
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            x1 = float(state[0])
            y1 = float(state[1])
            z1 = float(state[2])
            x2 = float(state[6])
            y2 = float(state[7])
            z2 = float(state[8])
            if i%skip_factor == 0:
                ax.plot([x1,x2], [y1,y2], [z1,z2], c='blue', marker='o', markersize=1, linestyle=':', linewidth=0.5, alpha=0.2)
            i = i + 1
            # ax.scatter(x1, y1, z1, s=10, c='black', marker='o')
            # ax.scatter(x2, y2, z2, s=10, c='orange', marker='o')

def plot_path(data_file, ax, lstyle, lcolor, llabel):
    x,y,z = read_traj(data_file)
    # print("path last state: ")
    # print(x[-1], y[-1], z[-1])
    ax.plot(x,y,z, c=lcolor, linestyle=lstyle, linewidth=1, label=llabel)
    # ax.scatter(x, y, z, s=30, c='black', marker='x')
    ax.scatter(x[0], y[0], z[0], s=40, c='black', marker='o', label="x0")

def plot_traj(data_file, ax, lstyle, lcolor, llabel, isCool=False):
    x,y,z = read_traj(data_file)
    print("traj last state: ")
    print(x[-1], y[-1], z[-1])
    ax.scatter(x[0], y[0], z[0], s=30, c='black', marker='o', label="x0")
    if(llabel == "rrt"):
        ax.scatter(x[-1], y[-1], z[-1], s=100, c='r', marker='*', label="xf")
    else:
        ax.scatter(x[-1], y[-1], z[-1], s=100, c=lcolor, marker='*', label="xf")
    if(isCool):
        n = len(x)

        # Define the color gradient (you can customize it)
        color_gradient = np.linspace(0, 1, n)

        # Prepare the points as a list of coordinates
        points = np.array([x, y, z]).T.reshape(-1, 1, 3)

        # Create a list of line segments
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Create a Line3DCollection with a colormap for the color gradient
        lc = Line3DCollection(segments, cmap=plt.get_cmap('cool'), norm=plt.Normalize(0, 1), label="traj")
        lc.set_array(color_gradient)

        # Add the Line3DCollection to the axes
        ax.add_collection3d(lc)
  
    else:
        ax.plot(x,y,z, c=lcolor, linestyle=lstyle, linewidth=1, label=llabel)


def plot_control_signals(data_file):
    # Define lists to store data
    u1 = []
    u2 = []
    u3 = []
    
    # Read data from the file
    with open(data_file, "r") as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            u1.append(float(state[0]))
            u2.append(float(state[1]))
            u3.append(float(state[2]))

    u1 = np.array(u1)
    u2 = np.array(u2)
    u3 = np.array(u3)
    t = np.cumsum(u3)
    t = np.insert(t, 0, 0)
    # Create a subplot with 2 rows and 1 column
    plt.figure(figsize=(8, 6))  # Adjust the figure size as needed

    # First subplot
    plt.subplot(2, 1, 1)
    plt.scatter(t[:-1], u1, label='u1', color = 'blue')
    for i in range(len(u3)):
        plt.plot([t[i], t[i+1]],[u1[i], u1[i]], color='blue')
    plt.xlabel('t')
    plt.ylabel('u1')
    plt.legend()
    plt.grid()

    # Second subplot
    plt.subplot(2, 1, 2)
    plt.scatter(t[:-1], u2, label='u2', color='orange')
    for i in range(len(u3)):
        plt.plot([t[i], t[i+1]],[u2[i], u2[i]], color='orange')
    plt.xlabel('t')
    plt.ylabel('u2')
    plt.legend()
    plt.grid()

    plt.tight_layout()  # Adjust subplot spacing

    plt.show()

def compare_trajs(traj_nlmpc, traj_rrt):
    time_step = 0.0001
    u_length = 108.4094
    km2m = 1000.0

    # Visualize 3D Traj
    ax = generate3Dplot()
    plot_traj(traj_nlmpc, ax, "--", "black", "nlmpc")
    plot_traj(traj_rrt, ax, "-", "blue", "rrt")
    ax.legend()
    plt.show()

    # State over time comparison
    ax = gernerate2Dplot(2,1)
    X1, Y1, Z1 = read_traj(traj_nlmpc)
    D1 = get_radius_distance(X1,Y1,Z1)*u_length*km2m
    t1 = np.arange(len(X1))*time_step
    X2, Y2, Z2 = read_traj(traj_rrt)
    D2 = get_radius_distance(X2,Y2,Z2)*u_length*km2m
    print("min radius comparison: ")
    print(min(D1), min(D2))
    t2 = np.arange(len(X2))*time_step
    ax[0].plot(t1, D1, label="nlmpc")
    ax[0].plot(t2, D2, label="rrt")
    ax[0].grid(True)
    ax[0].legend()
    ax[1].plot(t1, D1)
    ax[1].plot(t2, D2)
    ax[1].set_ylim(0, 2000)
    ax[1].axhline(500, linestyle="--", linewidth=1)
    ax[1].axhline(250, linestyle="--", linewidth=1)
    ax[1].grid(True)
    plt.show()

def compare_controls(traj_nlmpc, traj_rrt):
    time_step = 0.0001
    ax = gernerate2Dplot(2,1)
    U1, U2, U3 = read_traj(traj_rrt)
    T1 = np.cumsum(U3)
    T1 = np.insert(T1, 0, 0)
    ax[0].scatter(T1[:-1], U1, label='rrt', color = 'blue')
    for i in range(len(U3)):
        ax[0].plot([T1[i], T1[i+1]],[U1[i], U1[i]], color='blue')
    ax[1].scatter(T1[:-1], U2, color = 'blue')
    for i in range(len(U3)):
        ax[1].plot([T1[i], T1[i+1]],[U2[i], U2[i]], color='blue')

    U4, U5 = read_U(traj_nlmpc)
    T2 = np.arange(len(U4)) * time_step
    ax[0].plot(T2, U4, color="orange", label = 'nlmpc')
    ax[1].plot(T2, U5, color="orange")

    ax[0].legend()
    ax[0].grid(True)
    ax[1].grid(True)
    ax[0].set_ylabel("u1")
    ax[1].set_ylabel("u2")
    ax[1].set_xlabel("t")

    plt.show()


def generate3Dplot():
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Add 3D scatter points
    # ax.scatter(0, 0, 0, s=100, c='g', marker='o', label='Asteroid')  

    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_aspect('equal')
    return ax

def gernerate2Dplot(n_row, n_col):
    # Create a figure and two subplots using ax objects
    fig, ax = plt.subplots(n_row, n_col, figsize=(8, 6))
    return ax

def plot_traj_animation(data_file, ax, lstyle, lcolor, llabel, interval):
    x, y, z = read_traj(data_file)
    vx, vy, vz = read_vel(data_file)
    
    km2m = 1000
    x = x*km2m*unit_length
    y = y*km2m*unit_length
    z = z*km2m*unit_length
    vx = vx*km2m*unit_vel
    vy = vy*km2m*unit_vel
    vz = vz*km2m*unit_vel

    skip_factor = 2
    x = x[::skip_factor]
    y = y[::skip_factor]
    z = z[::skip_factor]
    
    # Find the minimum and maximum values for x, y, and z
    min_x, max_x = min(x), max(x)
    min_y, max_y = min(y), max(y)
    min_z, max_z = min(z), max(z)
    elevation = 13
    azimuth = -35
    ax.scatter(0, 0, 0, s=100, c='g', marker='o')
    ax.view_init(elev=elevation, azim=azimuth)

    # Choose a colormap from Matplotlib, e.g., 'viridis', 'rainbow', 'cool', etc.
    colormap = plt.get_cmap('viridis')  # Change 'viridis' to your desired colormap
    # Generate an array of colors
    colors = [colormap(i / len(x)) for i in range(len(x))]

    # Define the color gradient (you can customize it)
    color_gradient = np.linspace(0, 1, len(x))

    # Prepare the points as a list of coordinates
    points = np.array([x, y, z]).T.reshape(-1, 1, 3)

    # Create a list of line segments
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    #print("Total frame: ", len(x))
    N_show = int(len(x)/10)

    def update(frame):
        ax.cla()  # Clear the previous frame
        ax.scatter(0, 0, 0, s=100, c='g', marker='o')

        if(frame % N_show == 0):
            print(int(100*frame/len(x)))

        ss = segments[:frame]
        lc = Line3DCollection(ss, cmap=plt.get_cmap('cool'), norm=plt.Normalize(0, 1), label="traj")
        lc.set_array(color_gradient[:frame])
        ax.add_collection3d(lc)
        ax.scatter(x[frame], y[frame], z[frame], c='white', marker='*')

        ax.set_xlabel('X', color='white')
        ax.set_ylabel('Y', color='white')
        ax.set_zlabel('Z', color='white')
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_zlim(min_z, max_z)
        ax.set_aspect('equal')
        ax.set_title(f'(x{skip_factor}) Time Step {frame}/{len(x)}\nPos: {x[frame]:.1f},{y[frame]:.1f},{z[frame]:.1f} [m]\nVel: {vx[frame]:.3f},{vy[frame]:.3f},{vz[frame]:.3f} [m/s]', color='white')

    ani = FuncAnimation(plt.gcf(), update, frames=len(x), interval=interval, repeat=False)
    # plt.legend()  # Display legend
    # plt.show()
    video_name = "/Users/chko1829/src/SolarSailLanding/file_dump/videos/traj4.mp4"
    ani.save(video_name)
    print("saving to: ", video_name)

def generate_traj_animation(data_file, speed_interval=10):
    fig = plt.figure(figsize=(8,6))
    fig.set_facecolor("black")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor("black")
    ax.tick_params(axis='x', colors='white')
    ax.tick_params(axis='y', colors='white')    
    ax.tick_params(axis='z', colors='white')    
    lstyle = '-'  # Line style
    lcolor = 'b'  # Line color
    llabel = 'Trajectory'  # Line label
    plot_traj_animation(data_file, ax, lstyle, lcolor, llabel, speed_interval)

def reduce_list_by_skipping(input_list, k):
    if k <= 0:
        raise ValueError("k must be a positive integer")

    return input_list[::k]

def plot_graph_animation(segments, path):
    def init():
        ax.clear()
        ax.scatter(0, 0, 0, s=100, color='g')
        # Set the view angle
        elevation = 15  # Vertical viewing angle
        azimuthal = -7  # Horizontal viewing angle
        ax.view_init(elev=elevation, azim=azimuthal)

    def update(frame):
        if(frame < len(segments)):
            ax.clear()
            ax.scatter(0, 0, 0, s=100, color='g')
            for segment in segments[:frame + 1]:
                x, y, z = segment.T
                ax.plot(x, y, z, color='b', linewidth=0.5, linestyle='--', alpha=0.25)
                ax.scatter(x[0], y[0], z[0], s = 5, color='b', alpha=0.25)
                ax.scatter(x[-1], y[-1], z[-1], s = 5, color='orange', alpha=0.25)
        elif(frame < len(segments) + len(path[0])):
            index = frame - len(segments)
            x_p = path[0][index]
            y_p = path[1][index]
            z_p = path[2][index]
            ax.scatter(x_p, y_p, z_p, color='r', marker='x')
        else:
            ax.clear()
            ax.scatter(0, 0, 0, s=100, color='g')
            ax.scatter(path[0], path[1], path[2], color='r', marker='x')
            plot_traj("/Users/chko1829/src/SolarSailLanding/file_dump/trajectory.csv", ax, "-", "b", "traj", isCool=True)


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    anim = FuncAnimation(fig, update, frames=len(segments)+len(path[0])+1, init_func=init, repeat=False)
    # plt.show()
    video_name = "/Users/chko1829/src/SolarSailLanding/file_dump/videos/graph.mp4"
    anim.save(video_name)
    print("saving to: ", video_name)

def generate_graph_animation(graph_data_file, path_data_file):
    # create list of line segments
    segments= []
    # Read data from the file
    with open(graph_data_file, "r") as file:
        lines = file.readlines()
        i = 0
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            x1 = float(state[0])
            y1 = float(state[1])
            z1 = float(state[2])
            x2 = float(state[6])
            y2 = float(state[7])
            z2 = float(state[8])
            line_segment = np.array([[x1,y1,z1],[x2,y2,z2]])
            segments.append(line_segment)
    
    # reduce the list size
    N_total = len(segments)
    Max_edge = 200
    skip_factor = int(N_total/Max_edge)
    segements_reduced = reduce_list_by_skipping(segments, skip_factor)

    x_p, y_p, z_p = read_traj(path_data_file)
    path = []
    path.append(x_p)
    path.append(y_p)
    path.append(z_p)

    plot_graph_animation(segements_reduced, path)


#############################
# Global Variables
unit_length = 180.404
unit_length = 52.4409
unit_vel = 1.66344e-05
unit_vel = 8.73311e-06

def main():
    # traj_ompl = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/solution.txt"
    traj_ompl = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/trajectory.txt"
    # traj_ompl = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit1_valid1_ompl.txt"
    # traj_ompl = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_valid1_ompl.txt"
    # traj_ompl = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit3_valid1_ompl.txt"
    valid_ompl = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/validate_ompl.txt"
    plot_traj_ompl(valid_ompl)

if __name__ == "__main__":
    main()

