import argparse
import csv
import random
import math
import VirtualSpring
import HumanPathGenerator
import numpy as np
import matplotlib.pyplot as plt

desired_follow_distance = 2
desired_follow_angle = math.pi

robot_start = np.array([0, -1])

def linalg_norm(data):
    a, b = data[0]
    return numpy.linalg.norm(a - b, axis=1)

    
# plt.ion()

def test_path_following_algorithms(algorithm_nums, output_file, num_trials, show):
    hp = HumanPathGenerator.HumanPathGenerator()
    results = []
    #For each trial to run
    for i in range(num_trials):
        #Generate a random human path
        human_path = np.array(hp.generatePath(time_step=100))
        #Generate the desired_position array for comparison
        desired_position = np.array(list(map(lambda x: np.array([x[0] + (desired_follow_distance * math.sin(desired_follow_angle + x[2])), x[1] + (desired_follow_distance * math.cos(desired_follow_angle + x[2]))]), human_path)))
        robot = [robot_start]
        for algorithm_num in algorithm_nums:
            if algorithm_num == "spring":
                testAPF = VirtualSpring.VirtualSpring(desired_follow_distance=desired_follow_distance, desired_follow_angle=desired_follow_angle)
                error = [np.linalg.norm(robot_start - desired_position[0])]
                testAPF.updateEnvironment(human_path[0], [])
                testAPF.updateRobot(robot_start) #Initial robot position
                #Run the human path through the algorithm to get output velocities.
                for i in range(len(human_path)):
                    point = human_path[i]
                    robot_movement = testAPF.getRobotControlVelocity()
                    #Update the robot position based on ouput velocity
                    testAPF.updateRobot([testAPF.robot_position[0] + robot_movement[0], testAPF.robot_position[1] + robot_movement[1]])
                    robot.append(testAPF.robot_position)
                    #Log the error between the current robot poisition and the desired robot posiiton at this time step
                    error.append(np.linalg.norm(testAPF.robot_position - desired_position[i][0:2]))
                    testAPF.updateEnvironment(point, [])
                results.append(["spring", i, sum(error)/len(error), max(error)])
            elif algorithm_num == "astar":
                pass
        for x in human_path:
            plt.arrow(x[0], x[1], math.sin(x[2]) * .3, math.cos(x[2]) * .3)
        plt.plot([x[0] for x in human_path], [x[1] for x in human_path], '--o', label='human', c='green')
        plt.plot([x[0] for x in desired_position], [x[1] for x in desired_position], '--o', label='desired', c='red')
        plt.plot([x[0] for x in robot], [x[1] for x in robot], '--o', label='robot', c='blue')
        if show:
            plt.legend()
            plt.show()
            
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['algorithm', 'trial', 'avg_error', 'max_error'])
        for result in results:
            writer.writerow(result)

def main():
    parser = argparse.ArgumentParser(description='Test path following algorithms.')
    parser.add_argument('--algorithms', type=str, choices=["spring", "astar"], nargs='+', help='which path following algorithms to test ("spring", "astar")')
    parser.add_argument('--output', type=str, help='where to save the output CSV file')
    parser.add_argument('--trials', type=int, default=10, help='how many trials to run')
    parser.add_argument('--show', default=True, action=argparse.BooleanOptionalAction)
    args = parser.parse_args()

    algorithm_nums = args.algorithms
    output_file = args.output
    num_trials = args.trials
    show = args.show

    test_path_following_algorithms(algorithm_nums, output_file, num_trials, show)

if __name__ == "__main__":
    main()
