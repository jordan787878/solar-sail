import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.colors import Normalize
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import time
import pandas as pd
from matplotlib.colors import hsv_to_rgb
import matplotlib.patches as patches

def set_font_size(font_size=12):
    """
    Set the font size for all plots.

    Parameters:
    - font_size: Font size to be set (default is 12).
    """
    params = {'legend.fontsize': font_size,
              'axes.labelsize': font_size,
              'axes.titlesize': font_size,
              'xtick.labelsize': font_size,
              'ytick.labelsize': font_size,
              'axes.labelweight': 'bold',
              'axes.titleweight': 'bold'}
    
    plt.rcParams.update(params)

def read_traj(data_file):
    # Define lists to store data
    x = []
    y = []
    z = []
    vx = []
    vy = []
    vz = []
    
    # Read data from the file
    with open(data_file, "r") as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            x.append(float(state[0]))
            y.append(float(state[1]))
            z.append(float(state[2]))
            vx.append(float(state[3]))
            vy.append(float(state[4]))
            vz.append(float(state[5]))
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    vx = np.array(vx)
    vy = np.array(vy)
    vz = np.array(vz)
    return x,y,z,vx,vy,vz

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

# def read_U(data_file):
#     # Define lists to store data
#     x = []
#     y = []
    
#     # Read data from the file
#     with open(data_file, "r") as file:
#         lines = file.readlines()
#         for line in lines:
#             parts = line.strip().split()[0]  # Assuming data is space-separated
#             state = parts.split(",")
#             x.append(float(state[0]))
#             y.append(float(state[1]))
#     x = np.array(x)
#     y = np.array(y)
#     return x,y
def read_U(data_file):
    u1 = []
    u2 = []
    u3 = []
    
    # Read data from the file
    with open(data_file, "r") as file:
        lines = file.readlines()
        for line in lines:
            parts = line.strip().split()[0]  # Assuming data is space-separated
            state = parts.split(",")
            u1.append(float(state[6]))
            u2.append(float(state[7]))
            u3.append(float(state[8]))
    u1 = np.array(u1)
    u2 = np.array(u2)
    u3 = np.array(u3)
    return u1,u2,u3

def read_data(data_file):
    data = pd.read_csv(data_file, header=None)
    return data

def get_radius_distance(X, Y, Z):
    d = []
    for i in range(len(X)):
        x = X[i]
        y = Y[i]
        z = Z[i]
        d.append(np.sqrt(x*x+y*y+z*z))
    d = np.array(d)
    return d

def generate_traj_plot(ax, data_file, plot_color="black", plot_label="", plot_style="-", verbose=False, zoomin=False, plot_alpha=1):
    x,y,z,vx,vy,vz = read_traj(data_file)
    km2m = 1
    x = x*km2m*unit_length
    y = y*km2m*unit_length
    z = z*km2m*unit_length

    if(plot_color=="red"):
        qf = np.array([x[-1]/km2m/unit_length, y[-1]/km2m/unit_length, z[-1]/km2m/unit_length, vx[-1], vy[-1], vz[-1]])
        qf = np.round(qf, 2)
        print(qf)

    if(zoomin):
        d = get_radius_distance(x,y,z)
        unittimestep = 0.0001
        t = np.arange(0, len(d)) * unittimestep
        min_index = np.argmin(d)
        x_filtered = x[min_index-3:min_index+2]
        y_filtered = y[min_index-3:min_index+2]
        z_filtered = z[min_index-3:min_index+2]
        print(x_filtered)
        print(y_filtered)
        print(z_filtered)
        ax.plot(x_filtered, y_filtered, z_filtered, c=plot_color, marker='o', label=plot_label, linestyle = plot_style, alpha = plot_alpha)
    else:
        ax.plot(x, y, z, c=plot_color, label=plot_label, linestyle = plot_style, alpha = plot_alpha)
        if(verbose):
            ax.scatter(x[0], y[0], z[0],    c=plot_color, marker='o', label="xi", alpha = 1)
            ax.scatter(x[-1], y[-1], z[-1], c=plot_color, marker='*', label="xf", alpha = 1)

def generate_distance_plot(ax, data_file, plot_color="black", plot_label="", plot_style="-", verbose=False, zoomin=False, plot_alpha=1):
    x,y,z,vx,vy,vz = read_traj(data_file)
    km2m = 1000
    x = x*km2m*unit_length
    y = y*km2m*unit_length
    z = z*km2m*unit_length
    d = get_radius_distance(x,y,z)
    unittimestep = 0.0001/2
    t = np.arange(0, len(d)) * unittimestep
    ax.plot(t, d, c=plot_color, linestyle = plot_style, alpha = plot_alpha)
    # ax.set_xlim([0, 1.0])
    ax.set_xlabel('unit time')
    if(verbose):
        # ax.scatter(t[0], d[0], c=plot_color, marker='o', label="ri")
        # ax.scatter(t[-1], d[-1], c=plot_color, marker='*', label="rf")
        # plot the closest 
        min_index = np.argmin(d)
        ax.scatter(t[min_index], d[min_index], c=plot_color, marker='*', label=plot_label)
    if(zoomin):
        d = get_radius_distance(x,y,z)
        unittimestep = 0.0001
        t = np.arange(0, len(d)) * unittimestep
        min_index = np.argmin(d)
        t_filtered = t[min_index-3:min_index+2]
        d_filtered = d[min_index-3:min_index+2]
        ax.scatter(t_filtered, d_filtered, c=plot_color, marker='*')
    
def generate_utraj_plot(ax, data_file, t0 = 0, plot_color="black", plot_label="", plot_style="-"):
    u1, u2, u3 = read_U(data_file)
    u1 = u1[:-1]
    u2 = u2[:-1]
    # controltimestpe = 0.001
    # u3 = u3/controltimestpe
    u3 = u3[:-1]
    u3 = np.insert(u3, 0, 0)
    t = np.cumsum(u3) + t0
    ax[0].scatter(t[:-1], u1, color=plot_color)
    ax[0].plot([t[0], t[0+1]],[u1[0], u1[0]], color=plot_color)
    for i in range(len(u1)):
        ax[0].plot([t[i], t[i+1]],[u1[i], u1[i]], color=plot_color)
    ax[0].set_ylim([-np.pi/2, np.pi/2])
    
    ax[1].scatter(t[:-1], u2, color=plot_color)
    ax[1].plot([t[0], t[0+1]],[u2[0], u2[0]], color=plot_color)
    for i in range(len(u2)):
        ax[1].plot([t[i], t[i+1]],[u2[i], u2[i]], color=plot_color)
    ax[1].set_ylim([0, 2*np.pi])

    ax[1].set_xlabel("unit time")
    ax[0].set_ylabel("u1, rad")
    ax[1].set_ylabel("u2, rad")
    ax[0].grid(True)
    ax[1].grid(True)

def generate_utraj_plot_new(ax, data_file, t0 = 0, plot_color="black", plot_label="", plot_style="-"):
    u1, u2, u3 = read_U(data_file)
    u1 = u1[:-1]
    u2 = u2[:-1]
    # controltimestpe = 0.001
    # u3 = u3/controltimestpe
    u3 = u3[:-1]
    u3 = np.insert(u3, 0, 0)
    t = np.cumsum(u3) + t0
    ax[0].scatter(t[0], u1[0], color=plot_color)
    ax[0].scatter(t[:-1], u1, alpha=0.3, color=plot_color)
    ax[0].plot([t[0], t[0+1]],[u1[0], u1[0]], color=plot_color)
    for i in range(len(u1)):
        ax[0].plot([t[i], t[i+1]],[u1[i], u1[i]], alpha=0.3, color=plot_color)
    ax[0].set_ylim([-np.pi/2, np.pi/2])
    
    ax[1].scatter(t[0], u2[0], color=plot_color)
    ax[1].scatter(t[:-1], u2, alpha=0.3, color=plot_color)
    ax[1].plot([t[0], t[0+1]],[u2[0], u2[0]], color=plot_color)
    for i in range(len(u2)):
        ax[1].plot([t[i], t[i+1]],[u2[i], u2[i]], alpha=0.3, color=plot_color)
    ax[1].set_ylim([0, 2*np.pi])

    ax[1].set_xlabel("unit time")
    ax[0].set_ylabel("u1, rad")
    ax[1].set_ylabel("u2, rad")
    ax[0].grid(True)
    ax[1].grid(True)
    return t[1]

def generate_keepout(ax, j, center, radius, color_keepout = "red"):
    # Create a sphere-like scatter point
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
    if(j==0):
        ax.plot_surface(x, y, z, color=color_keepout, alpha=0.5, edgecolors="r", linewidth=0.0, label="keep-out")
    else:
        ax.plot_surface(x, y, z, color=color_keepout, alpha=0.5, edgecolors="r", linewidth=0.0)

def generate_keepout_2D(ax, j, center, radius, pos, r_max):
    X = pos[0]
    Y = pos[1]
    Z = pos[2]
    xc = center[0]
    yc = center[1]
    zc = center[2]
    print(xc, yc, zc)
    k = np.arange(1, len(X)+1)
    dX = X - xc
    dY = Y - yc
    dZ = Z - zc
    Distance = []
    for i in range(0, len(dX)):
        dist = np.sqrt(dX[i]*dX[i] + dY[i]*dY[i] + dZ[i]*dZ[i])
        Distance.append(dist)
    ax.plot(k, Distance, color='blue')
    ax.axhline(y=radius, color='r', linestyle='--')
    ax.set_ylim(0, 1.5*r_max)
    ax.grid(True)

def plot_x_traj(data_file, plot_title, plot_limit=-1, keepouts="Nan", eangle =45, aangle=45):
    x,y,z,vx,vy,vz = read_traj(data_file)

    km2m = 1
    x = x*km2m*unit_length
    y = y*km2m*unit_length
    z = z*km2m*unit_length
    vx = vx*km2m*unit_vel
    vy = vy*km2m*unit_vel
    vz = vz*km2m*unit_vel

    xf = x[-1]
    yf = y[-1]
    zf = z[-1]
    rf = np.sqrt(xf*xf + yf*yf + zf*zf)
    print(f'Pos Final: {xf:.3f},{yf:.3f},{zf:.3f}, Distance:{rf:.3f} [km]\nVel Final: {vx[-1]:.5f},{vy[-1]:.5f},{vz[-1]:.5f} [km/s]\n')

    # Create a 3D plot
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')

    # Set the view angle
    # elevation = 23  # Vertical viewing angle 33
    # azimuthal = 150  # Horizontal viewing angle 123
    ax.view_init(elev=eangle, azim=aangle)

    # Plot the 3D line
    ax.plot(x, y, z, c='black', label='Trajectory')
    ax.scatter(x[0], y[0], z[0], c='black', marker='o', label="xi")
    ax.scatter(xf, yf, zf, c='red', marker='*', label="xf")

    # Add 3D scatter points
    plot_asteroid(ax)

    # Keepout Zones
    if(keepouts != "Nan"):
        # print("keep out plot")
        km2m = 1000
        data_keepout = read_data(keepouts)
        x_keep = np.array(data_keepout[0])/km2m
        y_keep = np.array(data_keepout[1])/km2m
        z_keep = np.array(data_keepout[2])/km2m
        r_keep = np.array(data_keepout[3])/km2m
        # ax.scatter(x_keep, y_keep, z_keep, s=r_keep, c='red')
        for j in range(0,len(x_keep)):
            generate_keepout(ax, j, [x_keep[j], y_keep[j], z_keep[j]], r_keep[j])


    if(plot_limit > 0):
        ax.set_xlim([-plot_limit, plot_limit])
        ax.set_ylim([-plot_limit, plot_limit])
        ax.set_zlim([-plot_limit, plot_limit])

    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_aspect('equal')

    # Add a legend
    ax.legend()
    plt.title(plot_title)
    # Adjust x-axis label position
    ax.xaxis.labelpad = 0  # Adjust the padding as needed
    # plt.show()

def plot_keepout_2D(data_file, keepouts="Nan", N_keepout=0, cols=1):
    x,y,z,vx,vy,vz = read_traj(data_file)
    x = x*unit_length
    y = y*unit_length
    z = z*unit_length

    rows = int(N_keepout/cols)

    fig, axs = plt.subplots(rows, cols, figsize=(17, 5))
    if(keepouts != "Nan"):
        # print("keep out plot")
        data_keepout = read_data(keepouts)
        x_keep = np.array(data_keepout[0]/1000)
        y_keep = np.array(data_keepout[1]/1000)
        z_keep = np.array(data_keepout[2]/1000)
        r_keep = np.array(data_keepout[3]/1000)
        r_max = np.max(r_keep)
        # ax.scatter(x_keep, y_keep, z_keep, s=r_keep, c='red')
        for j in range(0,len(x_keep)):
            row = int(j/cols)
            col = j % cols
            generate_keepout_2D(axs[row,col], j, [x_keep[j], y_keep[j], z_keep[j]], r_keep[j], [x,y,z], r_max)
            if(col == 0):
                axs[row,col].set_ylabel('distance, km')
            if(row == int(N_keepout/cols)-1):
                axs[row,col].set_xlabel('time step')
            print(j)


def plot_traj_ompl(data_file, plot_title):
    data = np.loadtxt(data_file)

    x = np.array(data[:,0])
    y = np.array(data[:,1])
    z = np.array(data[:,2])
    km2m = 1000
    x = x*km2m*unit_length
    y = y*km2m*unit_length
    z = z*km2m*unit_length

    xf = x[-1]
    yf = y[-1]
    zf = z[-1]
    rf = np.sqrt(xf*xf + yf*yf + zf*zf)
    print(f'Pos Final: {xf:.1f},{yf:.1f},{zf:.1f}, Distance:{rf:.1f} [m]')

    # Create a 3D plot
    ax = generate3Dplot()

    # Set the view angle
    elevation = 7  # Vertical viewing angle
    azimuthal = -102  # Horizontal viewing angle
    ax.view_init(elev=elevation, azim=azimuthal)

    plot_asteroid(ax)

    # Plot the 3D line
    ax.plot(x, y, z, c='black', label='Trajectory')
    ax.scatter(x[0], y[0], z[0], c='black', marker='o', label="xi")
    ax.scatter(xf, yf, zf, c='red', marker='*', label="xf")

    # Add a legend
    ax.legend()
    # Show the 3D plot
    ax.set_aspect('equal')
    plt.title(plot_title)
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

def plot_asteroid(ax):
    # Sphere parameters
    radius = 0.25
    center = (0, 0, 0)
    phi, theta = np.mgrid[0.0:2.0*np.pi:20j, 0.0:np.pi:20j]
    # Spherical to Cartesian coordinates conversion
    x_ast = center[0] + radius * np.sin(theta) * np.cos(phi)
    y_ast = center[1] + radius * np.sin(theta) * np.sin(phi)
    z_ast = center[2] + radius * np.cos(theta)
    # Plot the 3D sphere
    #ax.plot_surface(x_ast, y_ast, z_ast, color='g', alpha=1, label="Asteroid")
    ax.plot_wireframe(x_ast, y_ast, z_ast, color='g', linewidth=0.5)

    radius = 1.0
    # Spherical to Cartesian coordinates conversion
    x_ast = center[0] + radius * np.sin(theta) * np.cos(phi)
    y_ast = center[1] + radius * np.sin(theta) * np.sin(phi)
    z_ast = center[2] + radius * np.cos(theta)
    # Plot the 3D sphere
    ax.plot_surface(x_ast, y_ast, z_ast, color='g', alpha=0.1)

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
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')

    # Set labels for the axes
    ax.set_xlabel('X, km')
    ax.set_ylabel('Y, km')
    ax.set_zlabel('Z, km')
    ax.set_aspect('equal')
    return ax

def gernerate2Dplot(n_row, n_col):
    # Create a figure and two subplots using ax objects
    fig, ax = plt.subplots(n_row, n_col, figsize=(8, 6))
    return ax

def plot_traj_animation(data_file, ax, lstyle, lcolor, llabel, interval):
    x, y, z, vx, vy, vz = read_traj(data_file)
    # vx, vy, vz = read_vel(data_file)
    
    km2m = 1000
    x = x*km2m*unit_length
    y = y*km2m*unit_length
    z = z*km2m*unit_length
    vx = vx*km2m*unit_vel
    vy = vy*km2m*unit_vel
    vz = vz*km2m*unit_vel

    skip_factor = 30
    x = x[::skip_factor]
    y = y[::skip_factor]
    z = z[::skip_factor]
    
    # Find the minimum and maximum values for x, y, and z
    min_x, max_x = min(x), max(x)
    min_y, max_y = min(y), max(y)
    min_z, max_z = min(z), max(z)
    elevation = 15  # Vertical viewing angle
    azimuthal = -7  # Horizontal viewing angle
    # ax.scatter(0, 0, 0, s=100, c='g', marker='o')
    ax.view_init(elev=elevation, azim=azimuthal)

    # plot the asteroid
    # plot_asteroid(ax)

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
        # plot_asteroid(ax)

        if(frame % N_show == 0):
            print(int(100*frame/len(x)))

        ss = segments[:frame]
        lc = Line3DCollection(ss, cmap=plt.get_cmap('cool'), norm=plt.Normalize(0, 1), label="traj")
        lc.set_array(color_gradient[:frame])
        ax.add_collection3d(lc)
        ax.scatter(x[frame], y[frame], z[frame], c='white', marker='*')
        r = np.sqrt(x[frame]*x[frame] + y[frame]*y[frame] + z[frame]*z[frame])

        ax.set_xlabel('X', color='white')
        ax.set_ylabel('Y', color='white')
        ax.set_zlabel('Z', color='white')
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_zlim(min_z, max_z)
        ax.set_aspect('equal')
        ax.set_title(f'(x{skip_factor}) Time Step {frame}/{len(x)}\nPos: {x[frame]:.1f},{y[frame]:.1f},{z[frame]:.1f}, Distance:{r:.1f} [m]\nVel: {vx[frame]:.3f},{vy[frame]:.3f},{vz[frame]:.3f} [m/s]', color='white')

    ani = FuncAnimation(plt.gcf(), update, frames=len(x), interval=interval, repeat=False)
    plt.legend()  # Display legend
    plt.show()
    # video_name = "/Users/chko1829/src/SolarSailLanding/file_dump/videos/orbit1_trajectory.mp4"
    # ani.save(video_name)
    # print("saving to: ", video_name)

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
    plt.show()
    # video_name = "/Users/chko1829/src/SolarSailLanding/file_dump/videos/graph.mp4"
    # anim.save(video_name)
    # print("saving to: ", video_name)

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
# unit_length = 180.404
unit_length = 52.4409
# unit_vel = 1.66344e-05
unit_vel = 8.73311e-06

def valid_mysetrrt_ode():
    orbit_index = 4
    traj_mysetrrt_valid_zeroU = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_valid_zeroU.csv"
    traj_mysetrrt_valid_dataU = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit4_valid_dataU.csv"

    plot_x_traj(traj_mysetrrt_valid_zeroU, 'mysetrrt ODE validation (zero U)')
    plt.show()
    plot_x_traj(traj_mysetrrt_valid_dataU, 'mysetrrt ODE validation (data U)')
    plt.show()

def valid_ompl_ode():
    orbit_index = 2
    traj_valid_zeroU = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_valid_zeroU.txt"
    traj_valid_dataU = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_valid_dataU.txt"
    plot_traj_ompl(traj_valid_zeroU, 'ompl ODE validation (zeroU)')
    # plot_traj_ompl(traj_valid_dataU, 'ompl ODE validation (dataU)')


def test_mysetrrt():
    orbit = 4
    traj = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit"+str(orbit)+"_mysetRRT_traj.csv"
    plot_x_traj(traj, "MySetRRT"+str(orbit))
    plt.show()

def test_mysetrrt_withkeepout():
    # testing code
    traj = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit2_mysetRRT_traj.csv"
    keepouts = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit2_keepouts.csv"

    # report code
    plot_path = "/Users/chko1829/src/SolarSailLanding/file_dump/plots/"
    traj = "/Users/chko1829/src/SolarSailLanding/file_dump/keepout/orbit2_mysetRRT_traj20.csv"
    keepouts = "/Users/chko1829/src/SolarSailLanding/file_dump/keepout/orbit2_keepouts20.csv"

    plot_x_traj(traj, '', keepouts=keepouts, eangle=7, aangle=-168)
    # plt.savefig(plot_path+"keepout_orbit2_setrrt_ko20.png", dpi=300, bbox_inches='tight', pad_inches=0.2)
    plt.show()

    # plot_keepout_2D(traj, keepouts=keepouts, N_keepout=4, cols=2)
    # # plt.savefig(plot_path+"keepout_orbit2_setrrt_ko20_2D.png", dpi=300, bbox_inches='tight', pad_inches=0.2)
    # plt.show()

    # plot_x_traj(traj, '', plot_limit=1.500, keepouts=keepouts)
    # plt.savefig(plot_path+"keepout_orbit2_setrrt(zoom-in)_ko6.png", dpi=300, bbox_inches='tight', pad_inches=0.2)
    # plt.show()


def test_mysetrrt_withnoise():
    traj1 = "/Users/chko1829/src/SolarSailLanding/file_dump/uncertainty/orbit2_omplRRT_traj_nom.csv"
    traj2 = "/Users/chko1829/src/SolarSailLanding/file_dump/uncertainty/orbit2_omplRRT_traj_noise.csv"
    ax = generate3Dplot()
    generate_traj_plot(ax, traj1, plot_color="black", plot_label="nom traj.", plot_style="-", verbose=False)
    generate_traj_plot(ax, traj2, plot_color="blue", plot_label="noise traj.", plot_style="--", verbose=True)
    plot_asteroid(ax)
    ax.view_init(13,-58)
    ax.legend()
    plt.savefig(plot_path+"orbit2_omplRRT_withnoise.png", dpi=300, bbox_inches='tight', pad_inches=0.2)
    plt.show()

    # plot_x_traj(traj, 'MySetRRT Orbit2')
    # plt.show()
    # plot_x_traj(traj, 'MySetRRT Orbit2 (zoom-in)', plot_limit=1.500)
    # plt.show()


def test_ompl():
    traj_valid =       "/Users/chko1829/src/SolarSailLanding/file_dump/for_report/orbit2_mysetRRT_valid.csv"

    traj = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplSST_traj.csv"
    utraj = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplSST_solution.csv"

    # position trajectory
    ax = generate3Dplot()
    generate_traj_plot(ax, traj_valid, plot_color="black", plot_label="nom traj.", plot_style=":")
    generate_traj_plot(ax, traj, plot_color='blue', plot_label="ctrl traj.", verbose=True)
    plot_asteroid(ax)
    ax.legend()
    plt.show()

    # control trajectory
    ax = gernerate2Dplot(n_row=2, n_col=1)
    generate_utraj_plot(ax, utraj)
    plt.show()


def plot_for_report():
    traj_valid =       "/Users/chko1829/src/SolarSailLanding/file_dump/for_report/orbit2_mysetRRT_valid.csv"
    traj_mysetrrt =    "/Users/chko1829/src/SolarSailLanding/file_dump/for_report/orbit2_mysetRRT_traj2.csv"
    # utraj_mysetrrt =    "/Users/chko1829/src/SolarSailLanding/file_dump/for_report/orbit2_mysetRRT_solution2.csv"
    utraj = "/Users/chko1829/src/SolarSailLanding/file_dump/for_report/orbit2_omplPDST_solution.csv"
    
    # ax = generate3Dplot()
    # generate_traj_plot(ax, traj_valid, plot_color="black", plot_label="nom traj.", plot_style=":")
    # generate_traj_plot(ax, traj_mysetrrt, plot_color="blue", plot_label="ctrl traj.", verbose=True)
    # plot_asteroid(ax)
    # ax.legend()
    # plt.show()
    # # control trajectory
    # ax = gernerate2Dplot(n_row=2, n_col=1)
    # generate_utraj_plot(ax, utraj_mysetrrt)
    # plt.show()

    # utraj_keepout = "/Users/chko1829/src/SolarSailLanding/file_dump/keepout/orbit2_mysetRRT_solution20.csv"
    # control trajectory (keep-out)
    ax = gernerate2Dplot(n_row=2, n_col=1)
    generate_utraj_plot(ax, utraj)
    plt.savefig(plot_path+"orbit2_ompl_pdst_utraj.png", dpi=300, bbox_inches='tight')
    plt.show()


def plot_benchmark():
    plot_path = "/Users/chko1829/src/SolarSailLanding/file_dump/plots/"
    algonames = ['Set-RRT','RRT','EST', 'KPIECE', 'PDST', 'SST', 'NLMPC pv', 'NLMPC peri.']

    metricnames = ['Comp. Time', 'Success', 'Traj. Length', 'Time of Flight', 'Graph Size']
    metricunits = [', sec', ', count', ', unit length', ', unit time', ', count']
    
    result_setrrt = "/Users/chko1829/src/SolarSailLanding/file_dump/MySetRRT/orbit2_mysetRRT_result.csv"
    result_rrt = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplRRT_result.csv"
    result_est = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplEST_result.csv"
    result_kp = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplKPIECE_result.csv"
    result_pdst = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplPDST_result.csv"
    result_sst = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplSST_result.csv"
    result_nlmpc_pv = "/Users/chko1829/src/SolarSailLanding/file_dump/from_matlab/orbit2_nlmpcPV_result.csv"
    result_nlmpc_oe = "/Users/chko1829/src/SolarSailLanding/file_dump/from_matlab/orbit2_nlmpcOE_result.csv"
    
    setrrt = pd.read_csv(result_setrrt, header=None, names=metricnames)
    rrt = pd.read_csv(result_rrt, header=None, names=metricnames)
    est = pd.read_csv(result_est, header=None, names=metricnames)
    kpiece = pd.read_csv(result_kp, header=None, names=metricnames)
    pdst = pd.read_csv(result_pdst, header=None, names=metricnames)
    sst = pd.read_csv(result_sst, header=None, names=metricnames)

    setrrt_filter = setrrt[setrrt["Success"] == 1]
    rrt_filter = rrt[rrt["Success"] == 1]
    est_filter = est[est["Success"] == 1]
    kpiece_filter = kpiece[kpiece["Success"] == 1]
    pdst_filter = pdst[pdst["Success"] == 1]
    sst_filter = sst[sst["Success"] == 1]
    sst_filter = sst_filter[sst_filter["Comp. Time"] < 61]


    # print(rrt_filter)
    # print(est_filter)
    # print(kpiece_filter)
    # print(pdst_filter)

    nlmpcpv = pd.read_csv(result_nlmpc_pv, header=None, names=metricnames)
    nlmpcoe = pd.read_csv(result_nlmpc_oe, header=None, names=metricnames)
    # Handle the special string representing negative infinity
    nlmpcpv.replace('-', float('-inf'), inplace=True)
    nlmpcoe.replace('-', float('-inf'), inplace=True)

    for i in range(0,5):
        metricname = metricnames[i]
        print(metricname)
        plt.figure(figsize=(10,6))

        if(metricname==metricnames[1]):
            plot_list = [setrrt[metricname], rrt[metricname], est[metricname], kpiece[metricname], pdst[metricname], sst[metricname], nlmpcpv[metricname], nlmpcoe[metricname]]
            success_rates = np.zeros(len(algonames))
            for j in range(0,len(algonames)):
                successrate = int(100*sum(np.array(plot_list[j]))/ len(np.array(plot_list[j])))
                success_rates[j] = successrate
            plt.bar(range(len(success_rates)), success_rates, color='blue')
            # Set the x-axis ticks and labels
            plt.xticks(range(len(success_rates)), algonames)  
            plt.title("Success Percentage, %")
            # Save the plot as a high-quality PNG file
            plt.savefig(plot_path+metricname+".png", dpi=300, bbox_inches='tight')
            plt.show()

        else:
            plot_list = [setrrt_filter[metricname], rrt_filter[metricname], est_filter[metricname], kpiece_filter[metricname], pdst_filter[metricname],  sst_filter[metricname], nlmpcpv[metricname], nlmpcoe[metricname]]
            max_y_value = pd.concat(plot_list, axis=0, ignore_index=True).max().max()
            plt.boxplot(plot_list, labels=algonames,
                    whiskerprops=dict(linewidth=2.0),  # Adjust the linewidth of the whiskers
                    boxprops=dict(linewidth=2.0),  # Adjust the linewidth of the box
                    medianprops=dict(color='red'))
            #plt.violinplot(plot_list)
            plt.title(metricname+metricunits[i])
            plt.ylim(0-max_y_value/100, max_y_value+max_y_value/100)
            # Save the plot as a high-quality PNG file
            plt.savefig(plot_path+metricname+".png", dpi=300, bbox_inches='tight')
            plt.show()


def test_replanning():
    plot_path = "/Users/chko1829/src/SolarSailLanding/file_dump/plots/"
    traj_valid =       "/Users/chko1829/src/SolarSailLanding/file_dump/for_report/orbit2_mysetRRT_valid.csv"
    traj = "/Users/chko1829/src/SolarSailLanding/file_dump/replanning/traj.csv"
    utraj = "/Users/chko1829/src/SolarSailLanding/file_dump/from_ompl/orbit2_omplSST_solution.csv"

    # position trajectory
    ax = generate3Dplot()
    generate_traj_plot(ax, traj_valid, plot_color="black", plot_label="nom traj.", plot_style=":")
    generate_traj_plot(ax, traj, plot_color='blue', plot_label="ctrl traj.", verbose=True)
    plot_asteroid(ax)
    ax.legend()
    plt.savefig(plot_path+"replan_traj.png", dpi=300, bbox_inches='tight')
    plt.show()

    # control trajectory
    n = 6
    cool_cmap = plt.get_cmap('cool')
    values = np.linspace(0, 1, n)
    rgb_colors = cool_cmap(values)
    print(values)


    ax = gernerate2Dplot(n_row=2, n_col=1)
    ax[0].set_facecolor('black')
    ax[1].set_facecolor('black')
    t0 = 0
    for i in range(0,n):
        print(i)
        utraj = "/Users/chko1829/src/SolarSailLanding/file_dump/replanning/solution"+str(i)+".csv"
        t0_new = generate_utraj_plot_new(ax, utraj, t0, plot_color=rgb_colors[i])
        t0 = t0_new
    plt.savefig(plot_path+"replan_utraj.png", dpi=300, bbox_inches='tight')
    plt.show()


def test_replanning_monte():
    # number of runs
    n = 100
    plot_path = "/Users/chko1829/src/SolarSailLanding/file_dump/plots/"
    traj_valid =       "/Users/chko1829/src/SolarSailLanding/file_dump/for_report/orbit2_mysetRRT_valid.csv"
    traj_path = "/Users/chko1829/src/SolarSailLanding/file_dump/replanning_monte/"

    # for bin plot
    w_s = []
    w_f = []
    index_f = []

    # position trajectory
    # ax = generate3Dplot()
    # plot_asteroid(ax)
    # generate_traj_plot(ax, traj_valid, plot_color="black", plot_label="nom traj.", plot_style=":")
    # plot colors
    cool_cmap = plt.get_cmap('cool')
    values = np.linspace(0, 1, n)
    rgb_colors = cool_cmap(values)
    for i in range(0,n):
        # traj = traj_path + str(i) + "/traj.csv"
        valuefile = traj_path + str(i) + "/values.csv"
        value_df = pd.read_csv(valuefile)
        success = value_df.loc[0, "Success"]
        W       = value_df.loc[0, "W"]
        if(success == 1):
            w_s.append(W)
            #generate_traj_plot(ax, traj, plot_color="blue", plot_label="ctrl traj.", plot_style='-', verbose=False, plot_alpha=0.1)
            
            # ax = generate3Dplot()
            # plot_asteroid(ax)
            # generate_traj_plot(ax, traj, plot_color="blue", plot_label="ctrl traj.", verbose=True, zoomin=False, plot_alpha=1.0)
            # ax.view_init(elev=22, azim=-113)
            # ax.set_aspect("equal")
            # plt.savefig(plot_path+"monte1/traj"+str(i)+".png", dpi=300, bbox_inches='tight', pad_inches=0.2)
        else:
            w_f.append(W)   
            index_f.append(i)
            # generate_traj_plot(ax, traj, plot_color="red", plot_label="ctrl traj.", plot_style='-', verbose=True, plot_alpha=0.5)
            # if(i==91):
            #     ax = generate3Dplot()
            #     plot_asteroid(ax)
            #     generate_traj_plot(ax, traj, plot_color="red", plot_label="ctrl traj.", verbose=True, zoomin=True, plot_alpha=0.5)
            #     ax.view_init(elev=20, azim=-140)
            #     ax.set_aspect("equal")
            #     plt.savefig(plot_path+"monte1/fail_traj"+str(i)+".png", dpi=300, bbox_inches='tight', pad_inches=0.2)
            #     plt.show()
    # ax.view_init(elev=22, azim=-113)
    # ax.set_aspect("equal")
    # plt.savefig(plot_path+"monte1/traj"+str(i)+".png", dpi=300, bbox_inches='tight', pad_inches=0.2)
    # plt.show()
    print(index_f)
    # return

    # distance plot
    # ax = gernerate2Dplot(n_row=1, n_col=1)
    # for i in range(0,n):
    #     traj = traj_path + str(i) + "/traj.csv"
    #     valuefile = traj_path + str(i) + "/values.csv"
    #     value_df = pd.read_csv(valuefile)
    #     success = value_df.loc[0, "Success"]
    #     W       = value_df.loc[0, "W"]
    #     if(success == 1):
    #         generate_distance_plot(ax, traj, plot_color="blue", plot_style=':', verbose=True, plot_alpha=0.1)
    #     else:
    #         generate_distance_plot(ax, traj, plot_color="red", plot_style=':', plot_label=str(i), verbose=True, plot_alpha=0.1)
    #     #if(i==5):
    #     #    generate_distance_plot(ax, traj, plot_color="red", plot_style=':', verbose=True, zoomin=True, plot_alpha=1.0)
    # # plt.savefig(plot_path+"replan_traj.png", dpi=300, bbox_inches='tight')
    # ax.set_ylabel("distance to goal, m")
    # # ax.axhline(y=250, color='r', linestyle='--', label='goal region')
    # # Define the rectangle properties
    # rectangle = patches.Rectangle((0, 0), 1, 250, linewidth=1, edgecolor='none', facecolor='green', alpha=0.3)
    # # Add the rectangle to the plot
    # ax.add_patch(rectangle)
    # ax.set_ylim(0, 5000)
    # ax.legend(loc='upper right')
    # ax.set_xlim(0, 1)
    # # plt.savefig(plot_path+"replan_monte_distance.png", dpi=300, bbox_inches='tight')
    # plt.show()

    # success/fail bin plot for different Ws
    w_s = np.array(w_s)
    w_f = np.array(w_f)
    ax = gernerate2Dplot(n_row=1, n_col=1)
    ax.hist(w_s, 10, histtype ='step', color = 'blue', fill=True, label="success")
    ax.hist(w_f, 10, histtype ='step', color = 'red', fill=True, label="fail")
    ax.legend()
    ax.set_title("Success Rate: " + str(np.round(100*len(w_s)/n,1)) + " %")
    ax.set_ylabel("counts")
    ax.set_xlabel("W")
    # plt.savefig(plot_path+"replan_monte_binplot.png", dpi=300, bbox_inches='tight')
    plt.show()




plot_path = "/Users/chko1829/src/SolarSailLanding/file_dump/plots/"
def main():

    set_font_size(font_size=12)

    # valid_mysetrrt_ode()

    test_mysetrrt()

    # test_mysetrrt_withkeepout()

    # test_mysetrrt_withnoise()

    # valid_ompl_ode()

    # test_ompl()

    # plot_for_report()

    # plot_benchmark()

    # test_replanning()

    # test_replanning_monte()

    # compare_planners()

##################################
    # test_accurate()

    # U_nlmpc = "/Users/chko1829/src/SolarSailLanding/file_dump/orbit4_u.csv"
    # traj_rrt = "/Users/chko1829/src/SolarSailLanding/file_dump/trajectory4.csv"
    # U_rrt = "/Users/chko1829/src/SolarSailLanding/file_dump/control_signals.csv"

    # graph_rrt = "/Users/chko1829/src/SolarSailLanding/file_dump/graph.csv"
    # path_rrt = "/Users/chko1829/src/SolarSailLanding/file_dump/path.csv"

    # plot_x_traj(traj, 'ODE validation (unforced)')
    # plot_traj_ompl(traj_valid_ompl, 'OMPL ODE validation (unforced)')
    
    # plot_x_traj(traj_nlmpc, 'NLMPC planner (and ODE validation forced)')
    # plot_traj_ompl(traj_valid_forced_ompl, 'NLMPC planner (and OMPL ODE validation forced)')
    
    # plot_x_traj(traj_rrt_ompl, 'OMPL RRT planner')
    # plot_x_traj(traj_mysetrrt, 'MySetRRT planner')

    # State Comparison
    # compare_trajs(traj_nlmpc, traj_rrt)

    # # # # Control Comparsion
    # compare_controls(U_nlmpc, U_rrt)

    # # # # RRT Visualization
    # create_plots(traj_rrt)
    # # plot_control_signals(U_rrt)

    # # Create a figure and 3D axis
    # generate_traj_animation(traj_rrt)
    # generate_traj_animation(traj_rrt_ompl)
    # traj = "/Users/chko1829/src/SolarSailLanding/file_dump/replanning_monte/3/traj.csv"
    # generate_traj_animation(traj)

    # generate_graph_animation(graph_rrt, path_rrt)


if __name__ == "__main__":
    main()

