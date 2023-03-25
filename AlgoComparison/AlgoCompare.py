import argparse
import csv
import random
import math
import VirtualSpring
import HumanPathGenerator
import numpy as np
import matplotlib.pyplot as plt
from AStarOpen import astar

desired_follow_distance = 2
desired_follow_angle = math.pi/4

obstacle_count = 2
max_obstacle_size = .3
min_obstacle_size = .1

def linalg_norm(data):
    a, b = data[0]
    return numpy.linalg.norm(a - b, axis=1)

def path_length(path):
    if(len(path) < 2):
        return 0
    length = 0
    for i in range(len(path) - 1):
        length += math.sqrt((path[i + 1][0] - path[i][0])**2 + (path[i + 1][1] - path[i][1])**2)
    return length

def test_path_following_algorithms(algorithm_nums, output_file, num_trials, show, obstacles):
    hp = HumanPathGenerator.HumanPathGenerator()
    results = []
    #For each trial to run
    for i in range(num_trials):
        #Generate a random human path
        human_path = np.array(hp.generatePath(time_step=10, path_length=1000))
        #Generate the desired_position array for comparison
        desired_position = np.array(list(map(lambda x: np.array([x[0] + (desired_follow_distance * math.sin(desired_follow_angle + x[2])), x[1] + (desired_follow_distance * math.cos(desired_follow_angle + x[2]))]), human_path)))
        # desired_position = np.array(list(map(lambda x: np.array([x[0] + (desired_follow_distance * math.sin(desired_follow_angle + human_path[0][2])), x[1] + (desired_follow_distance * math.cos(desired_follow_angle + human_path[0][2]))]), human_path)))
        robot_start = desired_position[0]
        robot = [robot_start]
        #If obstacles are enabled add one half-way through the desired trajectory
        obstacles_array = []
        if obstacles:
            for x in range(obstacle_count):
                obstacles_array.append([*desired_position[math.floor(len(desired_position) * ((x + 1) / (obstacle_count + 1)))], random.uniform(min_obstacle_size, max_obstacle_size)])
        for algorithm_num in algorithm_nums:
            if algorithm_num == "spring":
                testAPF = VirtualSpring.VirtualSpring(desired_follow_distance=desired_follow_distance, desired_follow_angle=desired_follow_angle)
                error = [np.linalg.norm(robot_start - desired_position[0])]
                testAPF.updateEnvironment(human_path[0], obstacles_array)
                testAPF.updateRobot(robot_start) #Initial robot position
                #Run the human path through the algorithm to get output velocities.
                for j in range(len(human_path)):
                    point = human_path[j]
                    #Log the error between the current robot poisition and the desired robot posiiton at this time step
                    error.append(np.linalg.norm(testAPF.robot_position - desired_position[j][0:2]))
                    robot_movement = testAPF.getRobotControlVelocity()
                    #Update the robot position based on ouput velocity
                    testAPF.updateRobot([testAPF.robot_position[0] + robot_movement[0], testAPF.robot_position[1] + robot_movement[1]])
                    robot.append(testAPF.robot_position)
                    testAPF.updateEnvironment(point, obstacles_array)
                results.append(["spring", i, sum(error)/len(error), max(error), path_length(robot)])
                plt.plot([x[0] for x in robot], [x[1] for x in robot], label='spring', c='blue')
            elif algorithm_num == "astar":
                robot_astar = [robot_start]
                error = [np.linalg.norm(robot_start - desired_position[0])]
                for j in range(len(desired_position)):

                    point = tuple(desired_position[j])
                    error.append(np.linalg.norm(robot_astar[-1] - desired_position[j][0:2]))
                    robot_path = astar(obstacles_array, tuple(robot_astar[-1]), point, limit=20)
                    if len(robot_path) > 0:
                        robot_astar.extend(list(zip(robot_path[0], robot_path[1])))
                results.append(["astar", i, sum(error)/len(error), max(error), path_length(robot_astar)])
                plt.plot([x[0] for x in robot_astar], [x[1] for x in robot_astar], label='astar', c='pink')
        for x in human_path:
            plt.arrow(x[0], x[1], math.sin(x[2]), math.cos(x[2]), head_width=.01)
        for obstacle in obstacles_array:
            circle1 = plt.Circle(obstacle[0:2], obstacle[2], color='r')
            plt.gca().add_patch(circle1)
        plt.plot([x[0] for x in human_path], [x[1] for x in human_path], label='human', c='green')
        plt.plot([x[0] for x in desired_position], [x[1] for x in desired_position], label='desired', c='red')

        if show:
            plt.legend()
            plt.axis('equal')
            plt.show()
            
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['algorithm', 'trial', 'avg_error', 'max_error', 'path_length'])
        for result in results:
            writer.writerow(result)

def main():
    parser = argparse.ArgumentParser(description='Test path following algorithms.')
    parser.add_argument('--algorithms', type=str, choices=["spring", "astar"], nargs='+', help='which path following algorithms to test ("spring", "astar")')
    parser.add_argument('--output', type=str, help='where to save the output CSV file')
    parser.add_argument('--trials', type=int, default=10, help='how many trials to run')
    parser.add_argument('--show', default=True, action=argparse.BooleanOptionalAction)
    parser.add_argument('--obstacles', default=True, action=argparse.BooleanOptionalAction)
    args = parser.parse_args()

    obstacles = args.obstacles #Set this to enable or disable obstacles on the robot's trajectory
    algorithm_nums = args.algorithms
    output_file = args.output
    num_trials = args.trials
    show = args.show

    test_path_following_algorithms(algorithm_nums, output_file, num_trials, show, obstacles)

if __name__ == "__main__":
    main()
