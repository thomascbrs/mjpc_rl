from build_release.libmjpc_rl_pywrap import loadData
import numpy as np

def plot_contact(data):
    """ Plot the main contact status.
    """
    fig, axs = plt.subplots(4, 1)
    names = ["FR", "FL", "HR", "HL"]

    dt = 0.002
    T = np.arange(0.,dt*len(data.foot_status[names[0]]), dt)
    for i,name in enumerate(names):
        ax = plt.subplot(4,1,i+1)

        pos_x = [pos[0]for pos in data.foot_position[name]]
        pos_z = [pos[2]for pos in data.foot_position[name]]
        max_z = max(pos_z)

        # Map 0 to "ground" and 1 to "flight" for plotting
        # status_labels = {0.8 * max_z: "ground", 0: "flight"}
        # contact_labels = [data.foot_status[name] for status in data.foot_status[name]]
        contacts = [0.8 * max_z if status == 0 else 0. for status in data.foot_status[name]]
        contacts_touch = [0.7 * max_z if status == 0 else 0. for status in data.foot_status[name]]

        # ax.plot(pos_x, contact_labels, "r-", label = "status_" + name)
        ax.plot(pos_x, contacts, "r-", label = "status_" + name)
        ax.plot(pos_x, contacts_touch, "g-", label = "touch_sens" + name)
        ax.plot(pos_x, pos_z, "bx-", label = "pos_" + name)
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
    order = [1,5,9,2,6,10,3,7,11,4,8,12]

    dt = 0.002
    T = np.arange(0.,dt*len(data.foot_velocity[names[0]]), dt)
    for i,name in enumerate(names):
        ax = plt.subplot(3,4,order[3*i])
        pos_ = [pos[0]for pos in data.foot_velocity[name]]
        ax.plot(T, pos_, "bx-", label = "vel_x")
        ax.set_title("velocity_x : " + name)

        ax = plt.subplot(3,4,order[3*i+1])
        pos_ = [pos[1]for pos in data.foot_velocity[name]]
        ax.plot(T, pos_, "bx-", label = "vel_y")
        ax.set_title("velocity_y : " + name)

        ax = plt.subplot(3,4,order[3*i+2])
        pos_ = [pos[2]for pos in data.foot_velocity[name]]
        ax.plot(T, pos_, "bx-", label = "vel_z")
        ax.set_title("velocity_z : " + name)

    # Adjust the vertical space between subplots
    plt.subplots_adjust(hspace=0.5)  # You can adjust the value as needed
    fig.suptitle("Foot Velocity")

    # Get the figure manager and set the window title
    fig_manager = plt.get_current_fig_manager()
    fig_manager.set_window_title("Foot Velocity")

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    plt.ion()

    # Load the data.
    data = loadData("/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin")

    plot_contact(data)
    plot_velocity(data)
    plt.show()
