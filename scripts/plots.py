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

        # ax.plot(pos_x, contact_labels, "r-", label = "status_" + name)
        ax.plot(pos_x, contacts, "r-", label = "status_" + name)
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
    fig_manager.set_window_title("Your Custom Window Title")

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    plt.ion()

    # Load the data.
    data = loadData("/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin")

    plot_contact(data)
    plt.show()
