from build_release.libmjpc_rl_pywrap import loadData
import numpy as np
import pinocchio as pin
from ndcurves import bezier
from copy import copy
from scripts.Filter import Filter, FilterMean


def plot_contact_MPCs(data):
    """ Plot the main contact status.
    """
    import example_robot_data
    robot = example_robot_data.load("a1")
    model = robot.model
    mdata = model.createData()

    fig, axs = plt.subplots(3, 4)
    names = ["FR", "FL", "HR", "HL"]
    frames = ["FR_foot", "FL_foot", "RR_foot", "RL_foot"]

    dt = data.dt_simu
    rfactor_mpc = 4
    rfactor_state = 10
    T = np.arange(0., dt * len(data.qpos), dt)

    # Define colors
    color_r = plt.cm.Reds(0.7)
    color_b = plt.cm.Blues(0.8)
    # Define the color values for replays
    cmap = plt.cm.Greys

    order_ = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]

    for i, mpc_data in enumerate(data.mpc_traj):
        # Timeline i-MPC
        t_start = i * data.k_mpc * dt
        t_end = t_start + data.dt_mpc * (data.horizon - 1)
        T_tmp = np.linspace(t_start, t_end, data.horizon)

        # Colors for horizon
        colors = np.linspace(0.6, 0.35, len(T_tmp))
        norm = plt.Normalize(colors.min(), colors.max())

        ##################
        # Angular position
        mpc_pos = {name: [] for name in frames}
        x = [state[:19] for state in mpc_data]
        for i, q in enumerate(x):
            q_tmp = copy(q)
            q_tmp[3:7] = q[4], q[5], q[6], q[3]
            q_tmp[7:] = robot.q0[7:] + q[7:]
            pin.forwardKinematics(model, mdata, np.array(q_tmp))
            for i, frame in enumerate(frames):
                frame_id = model.getFrameId(frame)
                oMf = pin.updateFramePlacement(model, mdata, frame_id)
                mpc_pos[frame].append(oMf.translation[:])

        for i, frame in enumerate(frames):
            ax = plt.subplot(3, 4, order_[3 * i])
            x = [pos[0] for pos in mpc_pos[frame]]
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

            ax = plt.subplot(3, 4, order_[3 * i + 1])
            x = [pos[1] for pos in mpc_pos[frame]]
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

            ax = plt.subplot(3, 4, order_[3 * i + 2])
            x = [pos[2] for pos in mpc_pos[frame]]
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

    T = np.arange(0., dt * len(data.foot_status[names[0]]), dt)
    for i, name in enumerate(names):

        ax = plt.subplot(3, 4, order_[3 * i])
        pos_x = [pos[0] for pos in data.foot_position[name]]
        max = np.max(pos_x)
        plot_state(ax, T, pos_x, color="b", label="pos_x")

        ax = plt.subplot(3, 4, order_[3 * i + 1])
        pos_x = [pos[1] for pos in data.foot_position[name]]
        max = np.max(pos_x)
        plot_state(ax, T, pos_x, color="b", label="pos_y")

        ax = plt.subplot(3, 4, order_[3 * i + 2])
        pos_x = [pos[2] for pos in data.foot_position[name]]
        max = np.max(pos_x)
        plot_state(ax, T, pos_x, color="b", label="pos_z")
        ax.legend()

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("Foot contact")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Foot contact")


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


def plot_state_filter(data):
    """ Plot the state.
    """
    fig, axs = plt.subplots(4, 3)
    names = ["FR", "FL", "HR", "HL"]
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]

    dt = 0.002
    T = np.arange(0., dt * len(data.contact_forces[names[0]]), dt)

    rpy = []
    for elt in data.qpos:
        rpy.append(pin.rpy.matrixToRpy(pin.Quaternion(elt[3], elt[4], elt[5], elt[6]).toRotationMatrix()))

    filtered_states = data.qpos_fil

    #################################
    # Position x,y,z first column.
    ax = plt.subplot(3, 4, order[0])
    x = [pos[0] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="x")
    x = [pos[0] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State x ")

    ax = plt.subplot(3, 4, order[1])
    x = [pos[1] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="y")
    x = [pos[1] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State y ")

    ax = plt.subplot(3, 4, order[2])
    x = [pos[2] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="z")
    x = [pos[2] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State z ")

    ###################
    # RPY - 2nd column.
    ax = plt.subplot(3, 4, order[3])
    x = [p[0] for p in rpy]
    ax.plot(T, x, "bx-", label="roll")
    x = [p[3] for p in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="roll-filtered")
    ax.set_title("Roll")

    ax = plt.subplot(3, 4, order[4])
    x = [p[1] for p in rpy]
    ax.plot(T, x, "bx-", label="pitch")
    x = [p[4] for p in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="roll-filtered")
    ax.set_title("pitch")

    ax = plt.subplot(3, 4, order[5])
    x = [p[2] for p in rpy]
    ax.plot(T, x, "bx-", label="yaw")
    x = [p[5] for p in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="roll-filtered")
    ax.set_title("yaw")


    filtered_states = data.qvel_fil

    ###############################
    # Linear velocity - 3rd column.
    ax = plt.subplot(3, 4, order[6])
    x = [pos[0] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_x")
    x = [pos[0] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State vel_x ")

    ax = plt.subplot(3, 4, order[7])
    x = [pos[1] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_y")
    x = [pos[1] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State vel_y ")

    ax = plt.subplot(3, 4, order[8])
    x = [pos[2] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_z")
    x = [pos[2] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State vel_z ")

    ###############################
    # Angular velocity - 4th column.
    ax = plt.subplot(3, 4, order[9])
    x = [pos[3] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wx")
    x = [pos[3] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State wx ")

    ax = plt.subplot(3, 4, order[10])
    x = [pos[4] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wy")
    x = [pos[4] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State wy ")

    ax = plt.subplot(3, 4, order[11])
    x = [pos[5] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wz")
    x = [pos[5] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State wz ")

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("States")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("States")


def plot_state_filterEval(data, wrapper):
    """ Plot the state.
    """
    fig, axs = plt.subplots(4, 3)
    names = ["FR", "FL", "HR", "HL"]
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]

    dt = 0.002
    T = np.arange(0., dt * len(data.contact_forces[names[0]]), dt)

    rpy = []
    for elt in data.qpos:
        rpy.append(pin.rpy.matrixToRpy(pin.Quaternion(elt[3], elt[4], elt[5], elt[6]).toRotationMatrix()))

    filter_pos = wrapper.getFilter()

    filtered_states = []
    for i,pos in enumerate(data.qpos):
        filtered_states.append(filter_pos.filter(np.concatenate([pos[:3],rpy[i]])))

    #################################
    # Position x,y,z first column.
    ax = plt.subplot(3, 4, order[0])
    x = [pos[0] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="x")
    x = [pos[0] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State x ")

    ax = plt.subplot(3, 4, order[1])
    x = [pos[1] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="y")
    x = [pos[1] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State y ")

    ax = plt.subplot(3, 4, order[2])
    x = [pos[2] for pos in data.qpos]
    ax.plot(T, x, "bx-", label="z")
    x = [pos[2] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State z ")

    ###################
    # RPY - 2nd column.
    ax = plt.subplot(3, 4, order[3])
    x = [p[0] for p in rpy]
    ax.plot(T, x, "bx-", label="roll")
    x = [p[3] for p in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="roll-filtered")
    ax.set_title("Roll")

    ax = plt.subplot(3, 4, order[4])
    x = [p[1] for p in rpy]
    ax.plot(T, x, "bx-", label="pitch")
    x = [p[4] for p in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="roll-filtered")
    ax.set_title("pitch")

    ax = plt.subplot(3, 4, order[5])
    x = [p[2] for p in rpy]
    ax.plot(T, x, "bx-", label="yaw")
    x = [p[5] for p in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="roll-filtered")
    ax.set_title("yaw")


    filter_vel = wrapper.getFilter()

    filtered_states = []
    for i,vel in enumerate(data.qvel):
        filtered_states.append(filter_vel.filter(vel[:6]))

    ###############################
    # Linear velocity - 3rd column.
    ax = plt.subplot(3, 4, order[6])
    x = [pos[0] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_x")
    x = [pos[0] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State vel_x ")

    ax = plt.subplot(3, 4, order[7])
    x = [pos[1] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_y")
    x = [pos[1] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State vel_y ")

    ax = plt.subplot(3, 4, order[8])
    x = [pos[2] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="vel_z")
    x = [pos[2] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State vel_z ")

    ###############################
    # Angular velocity - 4th column.
    ax = plt.subplot(3, 4, order[9])
    x = [pos[3] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wx")
    x = [pos[3] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State wx ")

    ax = plt.subplot(3, 4, order[10])
    x = [pos[4] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wy")
    x = [pos[4] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State wy ")

    ax = plt.subplot(3, 4, order[11])
    x = [pos[5] for pos in data.qvel]
    ax.plot(T, x, "bx-", label="wz")
    x = [pos[5] for pos in filtered_states]
    ax.plot(T, x, "r-",linewidth=4, label="x-filtered")
    ax.set_title("State wz ")

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("States")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("States")

def plot_contact_forces(data):
    fig, axs = plt.subplots(4, 3)
    names = ["FR", "FL", "HR", "HL"]
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]

    dt = 0.002
    T = np.arange(0., dt * len(data.contact_forces[names[0]]), dt)
    for i, name in enumerate(names):
        ax = plt.subplot(3, 4, order[3 * i])
        pos_ = [pos[0] for pos in data.contact_forces[name]]
        ax.plot(T, pos_, "b-", label="fc_x")
        pos_ = [pos[0] for pos in data.contact_forces_sensors[name]]
        ax.plot(T, pos_, "r-", label="fsensor_x")
        ax.set_title("Forces_x : " + name)

        ax = plt.subplot(3, 4, order[3 * i + 1])
        pos_ = [pos[1] for pos in data.contact_forces[name]]
        ax.plot(T, pos_, "b-", label="fc_y")
        pos_ = [pos[1] for pos in data.contact_forces_sensors[name]]
        ax.plot(T, pos_, "r-", label="fsensor_y")
        ax.set_title("Forces_y : " + name)

        ax = plt.subplot(3, 4, order[3 * i + 2])
        pos_ = [pos[2] for pos in data.contact_forces[name]]
        ax.plot(T, pos_, "b-", label="fc_z")
        pos_ = [pos[2] for pos in data.contact_forces_sensors[name]]
        ax.plot(T, pos_, "r-", label="fsensor_z")
        ax.set_title("Forces_z : " + name)

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("Contact forces")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Contact Forces")


def plot_state_simple(data):
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


def plot_MCP_horizon(ax, T, x, cmap, colors, norm, rfactor=1, label=""):
    ax.scatter(T[::rfactor], x[::rfactor], marker=".", s=5, cmap=cmap, c=colors[::rfactor], label=label)
    Tf = T[::rfactor]
    xf = x[::rfactor]
    rcolor = colors[::rfactor]
    for k in range(1, len(T[::rfactor])):
        ax.plot(Tf[k - 1:k + 1],
                xf[k - 1:k + 1],
                linestyle="-",
                color=cmap(norm(rcolor[k])),
                linewidth=2,
                markersize=0)


def plot_state(ax, T, x, color, label="", linestyle='-', marker='o', markersize=1, linewidth=1, rfactor=1):
    ax.plot(T[::rfactor],
            x[::rfactor],
            label=label,
            linewidth=linewidth,
            linestyle=linestyle,
            markersize=markersize,
            marker=marker,
            color=color)


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

        # Convert data.mpc_traj to a NumPy array
        mpc_data = np.array(mpc_data)

        # Replace values greater than 1e5 with zero
        mpc_data[mpc_data > 1e5] = 0
        mpc_data[mpc_data < -1e5] = 0

        mpc_data = mpc_data.tolist()

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
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

            x = [pos[k] for pos in data.qpos]
            plot_state(ax, T, x, color_b, rfactor_state)
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
            ax = plt.subplot(3, 4, order[k + 3])
            x = [state[k] for state in rpy_mpc]
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

            x = [pos[k] for pos in rpy_state]
            plot_state(ax, T, x, color_b, rfactor_state)
            ax.set_title("State " + names_pos[k + 3])

            if k == 1:
                x = [curve.get_pitch(t)[0] for t in T]
                ax.plot(T[::rfactor_state], x[::rfactor_state], "--", label="x", color=color_r, linewidth=2)

                x = [curve.get_pitch(t)[0] for t in T_tmp[T_tmp < 1]]
                ax.plot(T_tmp[T_tmp < 1][::rfactor_mpc],
                        x[::rfactor_mpc],
                        "--",
                        label="x",
                        color=color_r,
                        linewidth=2,
                        alpha=0.2)

        #####################
        # Pos/Ang velocities
        for k in range(6):
            ax = plt.subplot(3, 4, order[k + 6])
            x = [state[k + 19] for state in mpc_data]
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

            x = [pos[k] for pos in data.qvel]
            plot_state(ax, T, x, color_b, rfactor_state)

            if k < 3:  # Linear velocities
                x = [curve.curve.derivate(t, 1)[k] for t in T]
                ax.plot(T[::rfactor_state], x[::rfactor_state], "--", label="x", color=color_r, linewidth=2)

                # Ref for each MPC
                x = [curve.curve.derivate(t, 1)[k] for t in T_tmp[T_tmp < 1]]
                ax.plot(T_tmp[T_tmp < 1][::rfactor_mpc],
                        x[::rfactor_mpc],
                        "--",
                        label="x",
                        color=color_r,
                        linewidth=2,
                        alpha=0.2)

            # if k == 4:
            #     x = [curve.get_pitch(t)[1] for t in T]
            #     ax.plot(T[::rfactor_state], x[::rfactor_state], "--", label="x", color=color_r, linewidth=2)

            #     x = [curve.get_pitch(t)[1] for t in T_tmp[T_tmp < 1]]
            #     ax.plot(T_tmp[T_tmp < 1][::rfactor_mpc], x[::rfactor_mpc], "--", label="x", color=color_r, linewidth=2, alpha =0.2)

            ax.set_title("State " + names_vel[k])

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("States with MCP")


def plot_angular_position_MPCs(data):

    fig, axs = plt.subplots(3, 4)
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]
    names_pos = ["q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11"]
    actuator_names = dict()
    actuator_names = dict()
    actuator_names[0] = "FR_hip"
    actuator_names[1] = "FR_thigh"
    actuator_names[2] = "FR_calf"
    actuator_names[3] = "FL_hip"
    actuator_names[4] = "FL_thigh"
    actuator_names[5] = "FL_calf"
    actuator_names[6] = "RR_hip"
    actuator_names[7] = "RR_thigh"
    actuator_names[8] = "RR_calf"
    actuator_names[9] = "RL_hip"
    actuator_names[10] = "RL_thigh"
    actuator_names[11] = "RL_calf"

    dt = data.dt_simu
    rfactor_mpc = 4
    rfactor_state = 10
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
        colors = np.linspace(0.6, 0.35, len(T_tmp))
        norm = plt.Normalize(colors.min(), colors.max())

        ##################
        # Angular position
        for k in range(12):
            ax = plt.subplot(3, 4, order[k])
            x = [state[k + 7] for state in mpc_data]
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

    for k in range(12):
        ax = plt.subplot(3, 4, order[k])
        x = [state[k + 7] for state in data.qpos]
        plot_state(ax, T, x, color_b, label="measured", linewidth=2, markersize=1, rfactor=rfactor_state)
        ax.set_title("q" + str(k) + " : " + actuator_names[k])

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Angular position with MCP")


def plot_angular_velocities_MPCs(data):

    fig, axs = plt.subplots(3, 4)
    order = [1, 5, 9, 2, 6, 10, 3, 7, 11, 4, 8, 12]
    actuator_names = dict()
    actuator_names = dict()
    actuator_names[0] = "FR_hip"
    actuator_names[1] = "FR_thigh"
    actuator_names[2] = "FR_calf"
    actuator_names[3] = "FL_hip"
    actuator_names[4] = "FL_thigh"
    actuator_names[5] = "FL_calf"
    actuator_names[6] = "RR_hip"
    actuator_names[7] = "RR_thigh"
    actuator_names[8] = "RR_calf"
    actuator_names[9] = "RL_hip"
    actuator_names[10] = "RL_thigh"
    actuator_names[11] = "RL_calf"

    dt = data.dt_simu
    rfactor_mpc = 4
    rfactor_state = 10
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
        colors = np.linspace(0.6, 0.35, len(T_tmp))
        norm = plt.Normalize(colors.min(), colors.max())

        ##################
        # Angular position
        for k in range(12):
            ax = plt.subplot(3, 4, order[k])
            x = [state[k + 25] for state in mpc_data]
            plot_MCP_horizon(ax, T_tmp, x, cmap, colors, norm, rfactor_mpc)

    for k in range(12):
        ax = plt.subplot(3, 4, order[k])
        x = [state[k + 6] for state in data.qvel]
        plot_state(ax, T, x, color_b, label="measured", linewidth=2, markersize=1, rfactor=rfactor_state)
        ax.set_title("qv" + str(k) + " : " + actuator_names[k])

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Angular velocities with MCP")


class BezierRef():

    def __init__(self):
        P0 = [0.007, 0.0, 0.243]
        P1 = [0.656, 0.0, 0.009]
        P2 = [1.764, 0.0, 0.209]
        P3 = [0.756, 0.0, 0.938]
        P4 = [1.678, 0.0, 0.052]
        P5 = [2.801, 0.0, 0.324]
        self.curve = bezier(np.array([P0, P1, P2, P3, P4, P5]).T)

    def get_pitch(self, t):
        # Compute derivative of the curve wrt to x to retrieve pitch angle.
        dt = 0.01
        factor = 0.5

        # Current position
        p0 = np.zeros(3)
        # Backward (- dt)
        p1 = np.zeros(3)
        # Forward  (+ dt)
        p2 = np.zeros(3)

        if t - dt >= self.curve.min() and t + dt <= self.curve.max():
            p0 = self.curve(t)
            p1 = self.curve(t - dt)
            p2 = self.curve(t + dt)
        else:
            if t - dt < self.curve.min():
                # Beginning of the curve. Shift of dt.
                p0 = self.curve(t + dt)
                p1 = self.curve(t)
                p2 = self.curve(t + 2 * dt)
            else:
                # End of the curve. Shift of -dt.
                p0 = self.curve(t - dt)
                p1 = self.curve(t - 2 * dt)
                p2 = self.curve(t)

        # Compute pitch angle. Forward.
        pitch = 0. if p2[0] - p0[0] == 0. else (p2[2] - p0[2]) / (p2[0] - p0[0])
        pitch_backward = 0. if p0[0] - p1[0] == 0. else (p0[2] - p1[2]) / (p0[0] - p1[0])

        # Compute derivative.
        wpitch = (pitch - pitch_backward) / dt

        # Add factor
        pitch *= -factor
        wpitch *= -factor

        return pitch, wpitch

from enum import Enum

class FilterType(Enum):
    MOVING_AVERAGE = 1
    LOW_PASS = 2


class FilterWrapper:
    def __init__(self, order, fc, fs, period, dt, filterType):
        self.order = order
        self.fs = fs
        self.fc = fc
        self.period = period
        self.dt = dt

        # Example usage
        self.selected_filter = filterType

        if self.selected_filter == FilterType.MOVING_AVERAGE:
            print("Selected filter type: Moving Average")
        elif self.selected_filter == FilterType.LOW_PASS:
            print("Selected filter type: Low Pass")
        else:
            RuntimeError("Err")

    def getFilter(self):
        if self.selected_filter == FilterType.LOW_PASS:
            return Filter([self.fc]*6, self.fs, self.order)
        elif self.selected_filter == FilterType.MOVING_AVERAGE:
            return FilterMean(self.period, self.dt)


if __name__ == "__main__":

    import matplotlib.pyplot as plt
    plt.ion()

    curve = BezierRef()

    # Load the data.
    data = loadData("/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin")

    wrapper = FilterWrapper(order=1,
                            fc=2.,
                            fs=1/data.dt_simu,
                            period=0.1,
                            dt = data.dt_simu,
                            filterType=FilterType.LOW_PASS)

    # plot_contact_MPCs(data)
    # plot_contact(data)
    # plot_velocity(data)
    plot_state_filter(data)
    # plot_state_filterEval(data,wrapper)
    # plot_contact_forces(data)
    # plot_state_simple(data)
    # plot_state_mpc(data)
    # plot_angular_velocities_MPCs(data)
    # plot_angular_position_MPCs(data)
    plt.show()
