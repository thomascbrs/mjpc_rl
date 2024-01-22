from build_release.libmjpc_rl_pywrap import loadData
import numpy as np
import pinocchio as pin


def plot_contact(data):
    """ Plot the main contact status.
    """
    fig, axs = plt.subplots(4, 1)
    names = ["FR", "FL", "HR", "HL"]

    dt = 0.002
    T = np.arange(0., dt * len(data.foot_status[names[0]]), dt)
    for i, name in enumerate(names):
        ax = plt.subplot(4, 1, i + 1)

        pos_x = [pos[0] for pos in data.foot_position[name]]
        pos_z = [pos[2] for pos in data.foot_position[name]]
        max_z = max(pos_z)

        # Map 0 to "ground" and 1 to "flight" for plotting
        # status_labels = {0.8 * max_z: "ground", 0: "flight"}
        # contact_labels = [data.foot_status[name] for status in data.foot_status[name]]
        contacts = [0.8 * max_z if status == 0 else 0. for status in data.foot_status[name]]
        contacts_touch = [0.7 * max_z if status == 0 else 0. for status in data.foot_status[name]]

        # ax.plot(pos_x, contact_labels, "r-", label = "status_" + name)
        ax.plot(pos_x, contacts, "r-", label="status_" + name)
        ax.plot(pos_x, contacts_touch, "g-", label="touch_sens" + name)
        ax.plot(pos_x, pos_z, "bx-", label="pos_" + name)
        title = "Contact status foot " + name
        ax.set_title(title)
        # Customize y-axis ticks and labels
        # ax.set_yticks([0, 1])
        # ax.set_yticklabels(["ground", "flight"])

        ax.legend()

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("Foot contact")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Foot contact")


def plot_velocity(data):
    fig, axs = plt.subplots(4, 3)
    names = ["FR", "FL", "HR", "HL"]
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]

    dt = 0.002
    T = np.arange(0., dt * len(data.foot_velocity[names[0]]), dt)
    for i, name in enumerate(names):
        ax = plt.subplot(3, 4, order[3 * i])
        pos_ = [pos[0] for pos in data.foot_velocity[name]]
        ax.plot(T, pos_, "bx-", label="vel_x")
        ax.set_title("velocity_x : " + name)

        ax = plt.subplot(3, 4, order[3 * i + 1])
        pos_ = [pos[1] for pos in data.foot_velocity[name]]
        ax.plot(T, pos_, "bx-", label="vel_y")
        ax.set_title("velocity_y : " + name)

        ax = plt.subplot(3, 4, order[3 * i + 2])
        pos_ = [pos[2] for pos in data.foot_velocity[name]]
        ax.plot(T, pos_, "bx-", label="vel_z")
        ax.set_title("velocity_z : " + name)

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("Foot Velocity")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Foot Velocity")


def plot_contact_forces(data):
    fig, axs = plt.subplots(4, 3)
    names = ["FR", "FL", "HR", "HL"]
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]

    dt = 0.002
    T = np.arange(0., dt * len(data.contact_forces[names[0]]), dt)
    for i, name in enumerate(names):
        ax = plt.subplot(3, 4, order[3 * i])
        pos_ = [pos[0] for pos in data.contact_forces[name]]
        ax.plot(T, pos_, "bx-", label="fc_x")
        ax.set_title("Forces_x : " + name)

        ax = plt.subplot(3, 4, order[3 * i + 1])
        pos_ = [pos[1] for pos in data.contact_forces[name]]
        ax.plot(T, pos_, "bx-", label="fc_y")
        ax.set_title("Forces_y : " + name)

        ax = plt.subplot(3, 4, order[3 * i + 2])
        pos_ = [pos[2] for pos in data.contact_forces[name]]
        ax.plot(T, pos_, "bx-", label="fc_z")
        ax.set_title("Forces_z : " + name)

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("Contact forces")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Contact Forces")


def plot_state(data):
    """ Plot the state.
    """
    fig, axs = plt.subplots(4, 3)
    names = ["FR", "FL", "HR", "HL"]
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]

    dt = 0.002
    T = np.arange(0., dt * len(data.contact_forces[names[0]]), dt)

    #################################
    # Position x,y,z first column.
    ax = plt.subplot(3, 4, order[0])
    x = [pos[0] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="x")
    ax.set_title("State x ")

    ax = plt.subplot(3, 4, order[1])
    x = [pos[1] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="y")
    ax.set_title("State y ")

    ax = plt.subplot(3, 4, order[2])
    x = [pos[2] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="z")
    ax.set_title("State z ")

    ###################
    # RPY - 2nd column.
    rpy = []
    for elt in data.qpos:
        rpy.append(pin.rpy.matrixToRpy(pin.Quaternion(elt[3], elt[4], elt[5], elt[6]).toRotationMatrix()))
    ax = plt.subplot(3, 4, order[3])
    x = [p[0] for p in rpy]
    ax.plot(T, x, "bx-", label="roll")
    ax.set_title("Roll")

    ax = plt.subplot(3, 4, order[4])
    x = [p[1] for p in rpy]
    ax.plot(T, x, "bx-", label="pitch")
    ax.set_title("pitch")

    ax = plt.subplot(3, 4, order[5])
    x = [p[2] for p in rpy]
    ax.plot(T, x, "bx-", label="yaw")
    ax.set_title("yaw")

    ###############################
    # Linear velocity - 3rd column.
    ax = plt.subplot(3, 4, order[6])
    x = [pos[0] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_x")
    ax.set_title("State vel_x ")

    ax = plt.subplot(3, 4, order[7])
    x = [pos[1] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_y")
    ax.set_title("State vel_y ")

    ax = plt.subplot(3, 4, order[8])
    x = [pos[2] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_z")
    ax.set_title("State vel_z ")

    ###############################
    # Angular velocity - 4th column.
    ax = plt.subplot(3, 4, order[9])
    x = [pos[3] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wx")
    ax.set_title("State wx ")

    ax = plt.subplot(3, 4, order[10])
    x = [pos[4] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wy")
    ax.set_title("State wy ")

    ax = plt.subplot(3, 4, order[11])
    x = [pos[5] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wz")
    ax.set_title("State wz ")

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("States")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("States")


def plot_MCP_horizon(ax, T, x, cmap, colors, norm, rfactor=1):
    ax.scatter(T[::rfactor], x[::rfactor], marker=".", s=5, cmap=cmap, c=colors[::rfactor])
    Tf = T[::rfactor]
    xf = x[::rfactor]
    rcolor = colors[::rfactor]
    for k in range(1, len(T[::rfactor])):
        ax.plot(Tf[k - 1:k + 1], xf[k - 1:k + 1], linestyle="-", color=cmap(norm(rcolor[k])), linewidth=2, markersize=0)

def plot_state(ax,T,x,color,rfactor=1):
    ax.plot(T[::rfactor], x[::rfactor], "-", label="x", color=color, linewidth=4)

def plot_state_mpc(data):
    """ Plot the state.
    """
    fig, axs = plt.subplots(3, 4)
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]
    names_pos = ["x", "y", "z", "roll", "pitch", "yaw"]
    names_vel = ["vx", "vy", "vz", "wx", "wy", "wz"]

    dt = data.dt_simu
    rfactor_mpc = 2
    rfactor_state = 15
    T = np.arange(0., dt * len(data.qpos), dt)

    # Define colors
    color_r = plt.cm.Reds(0.7)
    color_b = plt.cm.Blues(0.8)
    # Define the color values for replays
    cmap = plt.cm.Greys

    for i, mpc_data in enumerate(data.mpc_traj):
        # Timeline i-MPC
        t_start = i * data.k_mpc * dt
        t_end = t_start + data.dt_mpc * (data.horizon - 1)
        T_tmp = np.linspace(t_start, t_end, data.horizon)

        # Colors for horizon
        colors = np.linspace(0.95, 0.45, len(T_tmp))
        norm = plt.Normalize(colors.min(), colors.max())

        ##################
        # Linear position
        for k in range(3):
            ax = plt.subplot(3, 4, order[k])
            x = [state[k] for state in mpc_data]
            plot_MCP_horizon(ax,T_tmp, x, cmap, colors, norm, rfactor_mpc)

            x = [pos[k] for pos in data.qpos]
            plot_state(ax,T,x,color_b, rfactor_state)
            ax.set_title("State " + names_pos[k])

        ###################
        # Angular position
        rpy_mpc = []
        for elt in mpc_data:
            rpy_mpc.append(pin.rpy.matrixToRpy(pin.Quaternion(elt[3], elt[4], elt[5], elt[6]).toRotationMatrix()))

        rpy_state = []
        for elt in data.qpos:
            rpy_state.append(pin.rpy.matrixToRpy(pin.Quaternion(elt[3], elt[4], elt[5], elt[6]).toRotationMatrix()))
        for k in range(3):
            ax = plt.subplot(3, 4, order[k+3])
            x = [state[k] for state in rpy_mpc]
            plot_MCP_horizon(ax,T_tmp, x, cmap, colors, norm, rfactor_mpc)

            x = [pos[k] for pos in rpy_state]
            plot_state(ax,T,x,color_b, rfactor_state)
            ax.set_title("State " + names_pos[k+3])

        #####################
        # Pos/Ang velocities
        for k in range(6):
            ax = plt.subplot(3, 4, order[k+6])
            x = [state[k+19] for state in mpc_data]
            plot_MCP_horizon(ax,T_tmp, x, cmap, colors, norm, rfactor_mpc)

            x = [pos[k] for pos in data.qvel]
            plot_state(ax,T,x,color_b, rfactor_state)
            ax.set_title("State " + names_vel[k])

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("States with MCP")


if __name__ == "__main__":

    import matplotlib.pyplot as plt
    plt.ion()

    # Load the data.
    data = loadData("/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin")

    # plot_contact(data)
    # plot_velocity(data)
    # plot_contact_forces(data)
    # plot_state(data)
    plot_state_mpc(data)
    plt.show()
