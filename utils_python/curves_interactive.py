import numpy as np
import matplotlib.pyplot as plt
from ndcurves import (bezier)
from matplotlib.patches import Circle
from matplotlib.widgets import Button

class BezierCurveEditor:
    def __init__(self, control_points):
        self.control_points = np.array(control_points)
        self.curve = bezier(self.control_points.T)
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Bezier Curve Editor")
        self.line, = self.ax.plot([], [], 'ro-', lw=2,markersize=8, markerfacecolor='none')
        self.control_points_scatter, = self.ax.plot([], [], 'bo', markersize=14)

        self.circle_radius = 0.2
        self.distance_selection = 0.3

        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.set_aspect('equal')


        self.dragging = False
        self.selected_point = None

        # Create permanent circles
        self.circles = [Circle((point[0], point[1]), radius=self.circle_radius, color='r', alpha=0.) for point in self.control_points]
        for circle in self.circles:
            self.ax.add_patch(circle)

        self.update_curve()

        # Button to print positions
        self.print_button_ax = self.fig.add_axes([0.01, 0.75, 0.1, 0.05])
        self.print_button = Button(self.print_button_ax, 'Print Positions')
        self.print_button.on_clicked(self.print_positions)

         # Button to add a control point
        self.add_button_ax = self.fig.add_axes([0.01, 0.85, 0.1, 0.05])
        self.add_button = Button(self.add_button_ax, 'Add Point')
        self.add_button.on_clicked(self.add_control_point)

        self.cid_press = self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.cid_release = self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.cid_motion = self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_xlim(-1., 4.5)
        self.ax.set_ylim(-1., 4.5)


        plt.show()

    def update_curve(self):
        t_values = np.linspace(0, 1, 25)
        curve_points = np.array([self.bezier(t) for t in t_values]).T
        self.line.set_xdata(curve_points[0])
        self.line.set_ydata(curve_points[1])
        self.control_points_scatter.set_xdata(self.control_points[:, 0])
        self.control_points_scatter.set_ydata(self.control_points[:, 1])

        # Draw circles around control points
        if self.selected_point is not None:
            point = self.control_points[self.selected_point]
            self.circles[self.selected_point].set_center((point[0], point[1]))

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw_idle()

    def bezier(self, t):
        self.curve = bezier(self.control_points.T)
        return self.curve(t)

    def on_press(self, event):
        if event.inaxes == self.ax:
            x, y = event.xdata, event.ydata
            distances = np.linalg.norm(self.control_points - np.array([x, y]), axis=1)
            selected_point = np.argmin(distances)
            if distances[selected_point] < self.distance_selection:
                self.selected_point = selected_point
                self.dragging = True
                self.circles[self.selected_point].set_alpha(0.3)

    def on_release(self, event):
        self.dragging = False
        # Remove circle around the selected point.
        if self.selected_point is not None:
            self.circles[self.selected_point].set_alpha(0.)

    def on_motion(self, event):
        if self.dragging:
            self.control_points[self.selected_point] = [event.xdata, event.ydata]
            self.update_curve()

    def add_control_point(self, event):
        new_point = [1.5,1.5]
        self.control_points = np.vstack([self.control_points, new_point])
        point = self.control_points[-1]
        self.circles.append(Circle((point[0], point[1]), radius=self.circle_radius, color='r', alpha=0.))
        self.ax.add_patch(self.circles[-1])
        self.update_curve()

    def print_positions(self, event):
        print("\n\n")
        for i, point in enumerate(self.control_points):
            print(f"P{i} = Eigen::Vector3d({point[0]:.3f}, 0.0, {point[1]:.3f});")
        print("\n\n")

if __name__ == "__main__":
    # Initial control points
    P0 = [0.007, 0.243]
    P1 = [1.25,  -0.6]
    P2 = [1.25,  3.]
    P3 = [4.,  0.173]
    initial_points = np.array([P0, P1, P2, P3])

    editor = BezierCurveEditor(initial_points)
