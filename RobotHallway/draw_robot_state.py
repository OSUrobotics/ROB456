#!/usr/bin/env python3

# Get the windowing/drawing packages
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGroupBox, QSlider, QLabel, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import Qt, QSize, QPoint
from PyQt5.QtGui import QPainter, QBrush, QPen, QFont, QColor

import numpy as np

# This bit of crazy code ensures that we can access the files in the two packages
from os import getcwd, chdir
import sys

# print(f"CWD: {getcwd()}\nPath: {sys.path}")
if "RobotHallway" in getcwd():
    chdir("..")
    cur_dir = getcwd()
    if cur_dir not in sys.path:
        sys.path.append(cur_dir)
    # print(f"Changed: {getcwd()}\nPath: {sys.path}")
else: 
    cur_dir = getcwd()
    if cur_dir not in sys.path:
        sys.path.append(cur_dir)
    # print(f"Changed: {getcwd()}\nPath: {sys.path}")


from RobotHallway.world_ground_truth import WorldGroundTruth
from RobotHallway.robot_ground_truth import RobotGroundTruth
from RobotHallway.robot_sensors import RobotSensors
from RobotHallway.bayes_filter import BayesFilter
from RobotHallway.kalman_filter import KalmanFilter
from RobotHallway.particle_filter import ParticleFilter

from gui_sliders import SliderFloatDisplay, SliderIntDisplay


# The main class for handling the robot drawing and geometry
class DrawRobotAndWalls(QWidget):
    def __init__(self, gui_world):
        super().__init__()

        # In order to get to the slider values
        self.gui = gui_world

        self.title = "Robot and Walls"

        # Window size
        self.top = 15
        self.left = 15
        self.width = 700
        self.height = 700

        # State/action text
        self.sensor_text = "No sensor"
        self.action_text = "No action"
        self.move_text = "No move"
        self.loc_text = "No location"

        # The world ground truth (doors)
        self.world_ground_truth = WorldGroundTruth()

        # For robot ground truth
        self.robot_ground_truth = RobotGroundTruth()

        # For querying doors/walls
        self.robot_sensors = RobotSensors()

        # For robot state estimation - the three different methods
        self.bayes_filter = BayesFilter()
        self.kalman_filter = KalmanFilter()
        self.particle_filter = ParticleFilter()

        # For keeping sampled error
        self.last_wall_sensor_noise = 0
        self.last_move_noise = 0

        # Height of prob
        self.draw_height = 0.6

        # Set geometry
        self.text = "None"
        self.init_ui()

    def init_ui(self):
        self.text = "No action or sensor"
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.show()

    # For making sure the window shows up the right size
    def minimumSizeHint(self):
        return QSize(self.width, self.height)

    # For making sure the window shows up the right size
    def sizeHint(self):
        return QSize(self.width, self.height)

    # Gaussian function
    @staticmethod
    def gaussian(x, mu, sigma):
        """Gaussian with given mu, sigma
        @param x - the input x value
        @param mu - the mu
        @param sigma - the standard deviation
        @return y = gauss(x) """
        return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(- (x - mu) ** 2 / (2 * sigma ** 2))

    # What to draw
    def paintEvent(self, event):
        qp = QPainter()
        qp.begin(self)
        self.draw_robot(qp)
        self.draw_wall(qp)
        if self.gui.which_filter == "Bayes":
            self.draw_probabilities(qp)
        elif self.gui.which_filter == "Kalman":
            self.draw_robot_gauss(qp)
        elif self.gui.which_filter == "Particle":
            self.draw_particles(qp)

        if self.gui.which_filter != "Bayes":
            self.draw_move_gauss(qp)
            self.draw_wall_gauss(qp)

        self.draw_world(qp)
        self.draw_sensor_action_text(qp, event)
        qp.end()

    # Put some text in the bottom left
    def draw_sensor_action_text(self, qp, _):
        qp.setPen(QColor(168,34,3))
        qp.setFont(QFont('Decorative', 10))
        #  Put sensor text in lower left...
        text_loc = QPoint(self.x_map(0.01), self.y_map(self.draw_height))
        qp.drawText(text_loc, self.sensor_text)
        # .. action text in lower right
        text_loc = QPoint(self.x_map(0.7), self.y_map(self.draw_height))
        qp.drawText(text_loc, self.action_text)
        # .. loc text in upper left
        text_loc = QPoint(self.x_map(0.01), self.y_map(self.draw_height + 0.025))
        qp.drawText(text_loc, self.loc_text)
        # .. move text in upper right
        text_loc = QPoint(self.x_map(0.7), self.y_map(self.draw_height + 0.025))
        qp.drawText(text_loc, self.move_text)

    def draw_wall(self, qp):
        pen = QPen(Qt.black, 4, Qt.SolidLine)
        brush = QBrush(Qt.CrossPattern)
        qp.setPen(pen)
        qp.setBrush(brush)

        qp.drawRect(self.x_map(0), self.y_map(1.0), self.x_map(1), self.in_pixels(0.2))

        pen.setColor(Qt.red)
        brush.setStyle(Qt.SolidPattern)
        qp.setPen(pen)
        qp.setBrush(brush)

        door_width = self.world_ground_truth.door_width
        for d in self.world_ground_truth.doors:
            qp.drawRect(self.x_map( d - door_width/2 ), self.y_map(0.95), self.in_pixels(door_width), self.in_pixels(0.15))

    def draw_probabilities(self, qp):
        pen = QPen(Qt.black, 1, Qt.SolidLine)
        qp.setPen(pen)

        div = 1.0 / self.bayes_filter.n_bins()
        for i in range(1, self.bayes_filter.n_bins()):
            qp.drawLine(self.x_map(i*div), self.y_map(0.0), self.x_map(i*div), self.y_map(self.draw_height*0.8))

        pen.setColor(Qt.blue)
        qp.setPen(pen)
        for i in range(1, self.bayes_filter.n_bins()):
            p = self.bayes_filter.probability(i)
            qp.drawLine(self.x_map(i * div), self.y_map(self.draw_height * p), self.x_map((i + 1) * div), self.y_map(self.draw_height * p))

    def draw_particles(self, qp):
        pen = QPen(Qt.black, 1, Qt.SolidLine)
        qp.setPen(pen)

        min_ws = np.min(self.particle_filter.weights)
        max_ws = np.max(self.particle_filter.weights)
        if np.isclose(max_ws, min_ws):
            max_ws = min_ws + 0.01
        for p, w in zip(self.particle_filter.particles, self.particle_filter.weights):
            h = 0.1 * (w - min_ws) / (max_ws - min_ws) + 0.05
            qp.drawLine(self.x_map(p), self.y_map(0.05), self.x_map(p), self.y_map(h))

    def draw_world(self, qp):
        pen = QPen(Qt.black, 3, Qt.SolidLine)
        qp.setPen(pen)

        qp.drawLine(self.x_map(0), self.y_map(0.0), self.x_map(1), self.y_map(0.0))
        qp.drawLine(self.x_map(0), self.y_map(0.0), self.x_map(0), self.y_map(self.draw_height))
        qp.drawLine(self.x_map(1), self.y_map(0.0), self.x_map(1), self.y_map(self.draw_height))

    def draw_robot_gauss(self, qp):
        pen = QPen(Qt.darkBlue, 1, Qt.DotLine)
        qp.setPen(pen)

        dx = np.linspace(0, 1, 200)
        dy = self.gaussian(dx, self.kalman_filter.location_mean(), self.kalman_filter.location_sigma())
        pts = []
        # Protect against sd set to zero/NaN
        max_y = np.max(dy)
        if max_y < 1e-6:
            max_y = 1e-6
        for x, y in zip(dx, dy):
            pts.append(QPoint(self.x_map(x), self.y_map(0.5 * y*self.draw_height/max_y)))
        for i in range(0, len(pts)-1):
            qp.drawLine(pts[i], pts[i+1])

    # Wall sensor distribution
    def draw_wall_gauss(self, qp):
        pen = QPen(Qt.gray, 1, Qt.DashLine)
        qp.setPen(pen)

        dx = np.linspace(0, 1, 200)
        dy = self.gaussian(dx, self.robot_ground_truth.robot_loc, self.gui.prob_query_wall_sigma.value())

        pts = []
        # Protect against sd set to zero/NaN
        max_y = np.max(dy)
        height = 0.1 / max_y
        for x, y in zip(dx, dy):
            pts.append(QPoint(self.x_map(x), self.y_map(self.draw_height + 0.05 + y*height)))
        for i in range(0, len(pts)-1):
            qp.drawLine(pts[i], pts[i+1])

        # Put a dashed line indicating last noise sample
        pen.setColor(Qt.red)
        pen.setWidth(1)
        qp.setPen(pen)
        qp.drawLine(QPoint(self.x_map(self.robot_ground_truth.robot_loc + self.last_wall_sensor_noise), self.y_map(self.draw_height + 0.1)),
                    QPoint(self.x_map(self.robot_ground_truth.robot_loc + self.last_wall_sensor_noise), self.y_map(self.draw_height + 0.045)))

    # Movement distribution
    def draw_move_gauss(self, qp):
        pen = QPen(Qt.gray, 1, Qt.DashLine)
        qp.setPen(pen)

        dx = np.linspace(0, 1, 200)
        loc = self.robot_ground_truth.robot_loc + self.gui.move_continuous_amount.value()
        dy = self.gaussian(dx, loc, self.gui.prob_move_sigma.value())

        pts = []
        # Protect against sd set to zero/NaN
        max_y = np.max(dy)
        height = 0.2
        for x, y in zip(dx, dy):
            pts.append(QPoint(self.x_map(x), self.y_map(y*height/max_y)))
        for i in range(0, len(pts)-1):
            qp.drawLine(pts[i], pts[i+1])

        # Put a dashed line indicating the last noise sample for move
        pen.setColor(Qt.red)
        pen.setWidth(2)
        qp.setPen(pen)
        qp.drawLine(QPoint(self.x_map(self.robot_ground_truth.robot_loc + self.last_move_noise), self.y_map(0)),
                    QPoint(self.x_map(self.robot_ground_truth.robot_loc + self.last_move_noise), self.y_map(0.075)))

    def draw_robot(self, qp):
        pen = QPen(Qt.darkMagenta, 2, Qt.SolidLine)
        qp.setPen(pen)

        x_i = self.x_map(self.robot_ground_truth.robot_loc)
        y_i = self.y_map(0.09)
        qp.drawLine(x_i-5, y_i, x_i+5, y_i)
        qp.drawLine(x_i, y_i-5, x_i, y_i+5)

    # Map from [0,1]x[0,1] to the width and height of the window
    def x_map(self, x):
        return int(x * self.width)

    # Map from [0,1]x[0,1] to the width and height of the window - need to flip y
    def y_map(self, y):
        return self.height - int(y * self.height) - 1

    def in_pixels(self, v):
        return int(v * self.height)

    def query_door_sensor(self):
        front_door_yn = self.door_sensor.is_in_front_of_door(self.world_ground_truth, self.robot_ground_truth)
        sensor_value = self.door_sensor.sensor_reading(self.world_ground_truth, self.robot_ground_truth)
        self.sensor_text = "Sensor reading: {}, actual: {}".format(sensor_value, front_door_yn)
        return sensor_value


class StateEstimationGUI(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setWindowTitle('State Estimation')

        # GUIDE Set this to whichever homework you're doing
        self.which_filter = "Bayes"
        # self.which_filter = "Kalman"
        # self.which_filter = "Particle"

        # Control buttons for the interface
        left_side_layout = self._init_left_layout_()
        middle_layout = self._init_middle_layout_()
        # right_side_layout = self._init_right_layout_()

        # The layout of the interface
        widget = QWidget()
        self.setCentralWidget(widget)

        # Three side-by-side panes
        top_level_layout = QHBoxLayout()
        widget.setLayout(top_level_layout)

        top_level_layout.addLayout(left_side_layout)
        top_level_layout.addLayout(middle_layout)

        # Make the variables match the sliders
        self.reset_simulation()
        self.random_doors()
        self.update_simulation_parameters()

        # So the sliders can update robot_scene
        SliderIntDisplay.gui = self
        SliderFloatDisplay.gui = self

    # Set up the left set of sliders/buttons (simulation parameters)
    def _init_left_layout_(self):
        # Reset buttons in upper left
        resets = QGroupBox('Resets')
        resets_layout = QVBoxLayout()

        # Number of bins to use for Bayes' filter
        if self.which_filter == "Bayes":
            self.n_bins = SliderIntDisplay('Number bins', 10, 30, 10)
            resets_layout.addWidget(self.n_bins)
        elif self.which_filter == "Particle":
            self.n_bins = SliderIntDisplay('Number bins', 10, 30, 10)
            resets_layout.addWidget(self.n_bins)
            self.n_samples = SliderIntDisplay('Number samples', 10, 1000, 100)
            resets_layout.addWidget(self.n_samples)

        # This one is valid no matter which filter we're doing
        reset_simulation_button = QPushButton('Reset simulation')
        reset_simulation_button.clicked.connect(self.reset_simulation)
        resets_layout.addWidget(reset_simulation_button)

        # This one is only valid for Bayes/Particle
        if not self.which_filter == "Kalman":
            self.n_doors = SliderIntDisplay('Number doors', 1, 6, 3)
            reset_random_doors_button = QPushButton('Random doors')
            reset_random_doors_button.clicked.connect(self.random_doors)
            resets_layout.addWidget(self.n_doors)
            resets_layout.addWidget(reset_random_doors_button)

        resets.setLayout(resets_layout)

        # Query state/do action buttons in the middle, left
        s_and_a = QGroupBox('State estimation: query state and do action')
        s_and_a_layout = QVBoxLayout()

        # Action/sensor buttons - update state estimation
        if not self.which_filter == "Bayes":
            query_wall_sensor_button = QPushButton('Query wall sensor')
            query_wall_sensor_button.clicked.connect(self.query_wall_sensor)
            s_and_a_layout.addWidget(query_wall_sensor_button)

        if not self.which_filter == "Kalman":
            query_door_sensor_button = QPushButton('Query door sensor')
            query_door_sensor_button.clicked.connect(self.query_door_sensor)
            s_and_a_layout.addWidget(query_door_sensor_button)

        if self.which_filter == "Bayes":
            move_left_button = QPushButton('Move left')
            move_left_button.clicked.connect(self.move_left)

            move_right_button = QPushButton('Move right')
            move_right_button.clicked.connect(self.move_right)

            s_and_a_layout.addWidget(move_left_button)
            s_and_a_layout.addWidget(move_right_button)
        else:
            move_continuous_button = QPushButton('Move continuous')
            move_continuous_button.clicked.connect(self.move_continuous)
            s_and_a_layout.addWidget(move_continuous_button)

            if self.which_filter == "Particle":
                importance_weight = QPushButton('Do importance weighting')
                importance_weight.clicked.connect(self.importance_weight)
                s_and_a_layout.addWidget(importance_weight)


        s_and_a.setLayout(s_and_a_layout)

        # The parameters of the world we're simulating
        parameters = QGroupBox('Simulation parameters')
        parameter_layout = QVBoxLayout()

        # Sensor parameters
        if self.which_filter != "Kalman":
            self.prob_see_door_if_door = SliderFloatDisplay('Prob see door if door', 0.01, 0.99, 0.8)
            self.prob_see_door_if_not_door = SliderFloatDisplay('Prob see door if not door', 0.01, 0.99, 0.1)

            parameter_layout.addWidget(self.prob_see_door_if_door)
            parameter_layout.addWidget(self.prob_see_door_if_not_door)

        if self.which_filter != "Bayes":
            self.prob_query_wall_sigma = SliderFloatDisplay('Prob distance sigma', 0.001, 0.3, 0.01)
            self.prob_move_sigma = SliderFloatDisplay('Prob move sigma', 0.001, 0.01, 0.005)

            parameter_layout.addWidget(self.prob_query_wall_sigma)
            parameter_layout.addWidget(self.prob_move_sigma)

        # Now actions
        if self.which_filter == "Bayes":
            self.prob_move_left_if_left = SliderFloatDisplay('Prob move left if left', 0.1, 0.85, 0.8)
            self.prob_move_right_if_left = SliderFloatDisplay('Prob move right if left', 0.0, 0.1, 0.05)
            self.prob_move_right_if_right = SliderFloatDisplay('Prob move right if right', 0.1, 0.85, 0.8)
            self.prob_move_left_if_right = SliderFloatDisplay('Prob move left if right', 0.0, 0.1, 0.05)

            parameter_layout.addWidget(self.prob_move_left_if_left)
            parameter_layout.addWidget(self.prob_move_right_if_left)
            parameter_layout.addWidget(self.prob_move_right_if_right)
            parameter_layout.addWidget(self.prob_move_left_if_right)

        # Continuous move amount
        if not self.which_filter == "Bayes":
            self.move_continuous_amount = SliderFloatDisplay('Amount move', -0.1, 0.1, 0.0)

            parameter_layout.addWidget(self.move_continuous_amount)

        parameters.setLayout(parameter_layout)

        # Put all the pieces in one box
        left_side_layout = QVBoxLayout()

        left_side_layout.addWidget(resets)
        left_side_layout.addStretch()
        left_side_layout.addWidget(s_and_a)
        left_side_layout.addStretch()
        left_side_layout.addWidget(parameters)

        return left_side_layout

    # Drawing screen and quit button
    def _init_middle_layout_(self):
        # The display for the robot drawing
        self.robot_scene = DrawRobotAndWalls(self)

        quit_button = QPushButton('Quit')
        quit_button.clicked.connect(app.exit)

        # Put them together, quit button on the bottom
        mid_layout = QVBoxLayout()

        mid_layout.addWidget(self.robot_scene)
        mid_layout.addWidget(quit_button)

        return mid_layout

    # Reset the number of bins, adjust the robot location
    def reset_simulation(self):
        if self.which_filter == "Bayes":
            self.robot_scene.bayes_filter.reset_probabilities(self.n_bins.value())
            self.robot_scene.robot_ground_truth._adjust_middle_of_bin(self.n_bins.value())
            self.robot_scene.world_ground_truth.random_door_placement(self.n_doors.value(), self.n_bins.value())
        elif self.which_filter == "Kalman":
            self.robot_scene.robot_ground_truth.reset_location()
            self.robot_scene.kalman_filter.reset_kalman()
        elif self.which_filter == "Particle":
            self.robot_scene.robot_ground_truth.reset_location()
            self.robot_scene.particle_filter.reset_particles(self.n_samples.value())

        self.robot_scene.repaint()

    def random_doors(self):
        try:
            self.robot_scene.world_ground_truth.random_door_placement(self.n_doors.value(), self.n_bins.value())
        except AttributeError:
            pass # Doing Kalman filter; no doors
        self.robot_scene.repaint()

    def update_simulation_parameters(self):
        """ Called whenever the simulation parameters change """
        if self.which_filter == "Bayes":
            self.robot_scene.robot_ground_truth.set_move_left_probabilities(self.prob_move_left_if_left.value(),
                                                                            self.prob_move_right_if_left.value())
            self.robot_scene.robot_ground_truth.set_move_right_probabilities(self.prob_move_left_if_right.value(),
                                                                             self.prob_move_right_if_right.value())
        if self.which_filter != "Kalman":
            self.robot_scene.robot_sensors.set_door_sensor_probabilites(self.prob_see_door_if_door.value(),
                                                                        self.prob_see_door_if_not_door.value())
        if self.which_filter != "Bayes":
            self.robot_scene.robot_ground_truth.set_move_continuos_probabilities(self.prob_move_sigma.value())
            self.robot_scene.robot_sensors.set_distance_wall_sensor_probabilities(self.prob_query_wall_sigma.value())

        self.repaint()

    def importance_weight(self):
        """ Put robot in the middle of the hallway"""
        self.robot_scene.particle_filter.resample_particles()
        self.repaint()

    def query_wall_sensor(self):
        # Do the sensor reading followed by the update
        dist_wall_z = self.robot_scene.robot_sensors.query_distance_to_wall(self.robot_scene.robot_ground_truth)
        if self.which_filter == "Kalman":
            self.robot_scene.kalman_filter.update_belief_distance_sensor(self.robot_scene.robot_sensors, dist_wall_z)
        elif self.which_filter == "Particle":
            self.robot_scene.particle_filter.calculate_weights_distance_wall(self.robot_scene.robot_sensors, dist_wall_z)

        # Update the drawing to show where the sample was taken from
        dist_wall_actual = self.robot_scene.robot_ground_truth.robot_loc
        self.robot_scene.last_wall_sensor_noise = dist_wall_z - dist_wall_actual
        self.robot_scene.loc_text = "Asked loc {0:0.2f}, got {1:0.2f}".format(dist_wall_actual, dist_wall_z)

        self.repaint()

    def query_door_sensor(self):
        # Do the sensor reading followed by the update
        returned_sensor_reading = self.robot_scene.robot_sensors.query_door(self.robot_scene.robot_ground_truth,
                                                                            self.robot_scene.world_ground_truth)
        if self.which_filter == "Bayes":
            self.robot_scene.bayes_filter.update_belief_sensor_reading(self.robot_scene.world_ground_truth,
                                                                       self.robot_scene.robot_sensors,
                                                                       returned_sensor_reading)
        elif self.which_filter == "Particle":
            self.robot_scene.particle_filter.calculate_weights_door_sensor_reading(self.robot_scene.world_ground_truth,
                                                                                   self.robot_scene.robot_sensors,
                                                                                   returned_sensor_reading)

        b_was_door = self.robot_scene.world_ground_truth.is_location_in_front_of_door(self.robot_scene.robot_ground_truth.robot_loc)
        self.robot_scene.sensor_text = f"Door {b_was_door}, got {returned_sensor_reading}"

        self.repaint()

    def move_left(self):
        # Try to move the robot left
        step_size = 1.0 / self.n_bins.value()
        step = self.robot_scene.robot_ground_truth.move_left(step_size)
        if step < 0:
            self.robot_scene.action_text = "Asked move left, moved left"
        elif step > 0:
            self.robot_scene.action_text = "Asked move left, moved right"
        else:
            self.robot_scene.action_text = "Asked move left, did not move"

        # Update the state estimation
        self.robot_scene.bayes_filter.update_belief_move_left(self.robot_scene.robot_ground_truth)

        self.repaint()

    def move_right(self):
        # Try to move the robot right
        step_size = 1.0 / self.n_bins.value()
        step = self.robot_scene.robot_ground_truth.move_right(step_size)
        if step > 0:
            self.robot_scene.action_text = "Asked move right, moved right"
        elif step < 0:
            self.robot_scene.action_text = "Asked move right, moved left"
        else:
            self.robot_scene.action_text = "Asked move right, did not move"

        self.robot_scene.bayes_filter.update_belief_move_right(self.robot_scene.robot_ground_truth)

        self.repaint()

    def reset_kalman(self):
        self.robot_scene.kalman_filter.reset_kalman()
        self.robot_scene.last_wall_reading = self.robot_scene.robot_ground_truth.robot_loc
        self.robot_scene.last_move_request = self.robot_scene.robot_ground_truth.robot_loc

        self.repaint()

    def query_wall_sensor_button_kalman(self):
        # Query the wall sensor
        dist_wall_z = self.robot_scene.robot_sensors.query_distance_to_wall(self.robot_scene.robot_ground_truth)
        dist_wall_actual = self.robot_scene.robot_ground_truth.robot_loc
        self.robot_scene.last_wall_sensor_noise = dist_wall_actual - dist_wall_z
        self.robot_scene.loc_text = "Asked loc {0:0.2f}, got {1:0.2f}".format(dist_wall_actual, dist_wall_z)
        self.robot_scene.kalman_filter.update_belief_distance_sensor(self.robot_scene.world_ground_truth, dist_wall_z)

        self.repaint()

    def move_continuous(self):
        # Try to move the robot by the amount in the slider
        amount_requested = self.move_continuous_amount.value()
        amount = self.robot_scene.robot_ground_truth.move_continuous(amount_requested)
        self.robot_scene.last_move_noise = amount - amount_requested
        self.robot_scene.move_text = "Asked move {0:0.4f}, moved {1:0.4f}".format(amount_requested, amount)
        if self.which_filter == "Kalman":
            self.robot_scene.kalman_filter.update_continuous_move(self.robot_scene.robot_ground_truth,
                                                                  self.move_continuous_amount.value())
        elif self.which_filter == "Particle":
            self.robot_scene.particle_filter.update_particles_move_continuous(self.robot_scene.robot_ground_truth,
                                                                              self.move_continuous_amount.value())

        self.repaint()

    def draw(self, _):
        self.robot_scene.draw(self.which_filter)


if __name__ == '__main__':
    app = QApplication([])

    gui = StateEstimationGUI()

    gui.show()

    app.exec_()
