"""
Main file for running the simulator.
"""

import csv
import json

from environment import Environment
from robot import Robot
from utils import Position, Pose, Landmark, Bounds

if __name__ == "__main__":
    # set up the environment
    dimensions = Bounds(x_min=0.0, x_max=10.0, y_min=0.0, y_max=10.0)
    dt = 0.1
    obstacles = [
        Bounds(x_min=2.0, x_max=3.5, y_min=2.0, y_max=6.0),
        Bounds(x_min=6.0, x_max=8.5, y_min=1.0, y_max=2.5),
    ]
    landmarks = [
        Landmark(Position(1.0, 1.0), id=1),
        Landmark(Position(8.0, 8.0), id=2),
        Landmark(Position(1.0, 8.5), id=3),
    ]
    initial_robot_pose = Pose(Position(0.5, 0.5), theta=0.0)

    env = Environment(
        dimensions,
        dt,
        obstacles,
        landmarks,
        initial_robot_pose,
    )

    # set up the robot
    robot = Robot(env)

    # set up timekeeping
    total_seconds = 10.0
    total_timesteps = total_seconds / env.DT

    # set up logging
    ground_truth_history = []
    sensor_data_history = []

    # set up input filepath and output filepaths
    input_commands_filepath = "input/vel_cmd_example.csv"
    output_ground_truth_filepath = "output/ground_truth.csv"
    output_sensor_data_filepath = "output/sensor_data.csv"

    # open up the instructions, pop the first
    with open(input_commands_filepath, "r", newline="") as cmd:
        reader = csv.reader(cmd)
        commands = []
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            time_s, lin_vel, ang_vel = row
            commands.append((float(time_s), float(lin_vel), float(ang_vel)))

        commands.sort(key=lambda x: x[0])
        cmd_index = 0
        current_lin = 0.0
        current_ang = 0.0

        # iterate through each timestep
        for step in range(int(total_timesteps) + 1):
            current_time = step * env.DT

            ground_truth_history.append(env.take_state_snapshot())

            sensor_data_history.append(robot.take_sensor_measurements())

            while cmd_index < len(commands) and commands[cmd_index][0] <= current_time:
                _, current_lin, current_ang = commands[cmd_index]
                cmd_index += 1

            robot.robot_step_differential(current_lin, current_ang)

    # at the end, write the histories into output files
    with open(output_ground_truth_filepath, "w", newline="") as gt_data:
        writer = csv.writer(gt_data)
        writer.writerow(["time", "robot_pose", "landmark_proximities"])
        for entry in ground_truth_history:
            writer.writerow(
                [
                    entry["time"],
                    json.dumps(entry["robot_pose"]),
                    json.dumps(entry["landmark_proximities"]),
                ]
            )

    with open(output_sensor_data_filepath, "w", newline="") as sensor_data:
        writer = csv.writer(sensor_data)
        writer.writerow(["time", "measurements"])
        for entry in sensor_data_history:
            writer.writerow(
                [
                    entry["time"],
                    json.dumps(entry["measurements"]),
                ]
            )
