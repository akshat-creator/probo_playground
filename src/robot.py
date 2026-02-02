"""
A simulated robotic agent with teleoperation and sensing capabilities.

The Robot class models the robotic agent that explores the world. The robot is remote-controlled by angular and linear velocity commands read from an external file. The robot can execute motor commands to move, and can sense both externally (GPS, landmarks, obstacles) and internally (odometry, IMU).
"""

import math
import random

from environment import Environment
from sensors import SensorInterface


class Robot:
    """
    A class that models a simulated robotic agent.

    Attributes:
        env: the environment this robot is operating in
        sensors: list of all robot sensors
    """

    def __init__(self, env: Environment):
        """
        Initialize an instance of the Robot class.

        Args:
            env: the environment this robot is operating in
        """
        # TODO: set the environment property to the parameter value
        self.env = env
        # TODO: initialize the sensors property as an empty list
        self.sensors = []
        self.last_lin_vel = 0.0
        self.last_ang_vel = 0.0
        self.last_x_vel = 0.0
        self.last_y_vel = 0.0

    def robot_step_differential(self, lin_vel: float, ang_vel: float):
        """
        Differential-drive mode. Given forward linear and angular velocities, determine the robot's change in x, y, and heading and apply those changes in the environment.

        Args:
            lin_vel: input linear velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """
        # TODO: fill in the function
        lin_vel_noisy = random.gauss(lin_vel, 0.05)
        ang_vel_noisy = random.gauss(ang_vel, 0.03)

        theta = self.env.robot_pose.theta
        dx = lin_vel_noisy * self.env.DT * math.cos(theta)
        dy = lin_vel_noisy * self.env.DT * math.sin(theta)
        dtheta = ang_vel_noisy * self.env.DT

        self.last_lin_vel = lin_vel_noisy
        self.last_ang_vel = ang_vel_noisy
        self.env.robot_step(dx, dy, dtheta)

        return dx, dy, dtheta

    def robot_step_translational(self, x_vel: float, y_vel: float, ang_vel: float):
        """
        Swerve-drive mode. Given x, y, and angular velocities, determine the robot's change in x, y, and heading and apply those changes in the environment.

        Args:
            x_vel: input x velocity command
            y_vel: input y velocity command
            ang_vel: input angular velocity command

        Returns:
            dx: change in x position
            dy: change in y position
            d-theta: change in heading
        """
        # TODO: fill in the function
        x_vel_noisy = random.gauss(x_vel, 0.05)
        y_vel_noisy = random.gauss(y_vel, 0.05)
        ang_vel_noisy = random.gauss(ang_vel, 0.03)

        dx = x_vel_noisy * self.env.DT
        dy = y_vel_noisy * self.env.DT
        dtheta = ang_vel_noisy * self.env.DT

        self.last_x_vel = x_vel_noisy
        self.last_y_vel = y_vel_noisy
        self.last_ang_vel = ang_vel_noisy
        self.env.robot_step(dx, dy, dtheta)

        return dx, dy, dtheta

    def take_sensor_measurements(self):
        """
        Return noisy sensor readings of the environment at this timestep, including data from all sensors, in a table format.
        """
        # TODO: fill in the function
        measurements = []
        current_time = self.env.time

        for sensor in self.sensors:
            if (current_time - sensor.last_meas_t) >= sensor.interval:
                sample = sensor.sample()
                sensor.last_meas_t = current_time
                measurements.append(
                    {
                        "sensor": sensor.name,
                        "measurement": sample,
                    }
                )

        return {
            "time": current_time,
            "measurements": measurements,
        }
