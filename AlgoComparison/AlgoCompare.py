import argparse
import csv
import random
import math
import MultiAgentAPF
import HumanPathGenerator
import numpy as np
import matplotlib.pyplot as plt

desired_follow_distance = 1
desired_follow_angle = math.pi

robot_start = np.array([0, -1])

def linalg_norm(data):
    a, b = data[0]
    return numpy.linalg.norm(a - b, axis=1)

    
# plt.ion()

def test_path_following_algorithms(algorithm_nums, output_file, num_trials):
    hp = HumanPathGenerator.HumanPathGenerator()
    results = []
    for i in range(num_trials):
        human_path = hp.generatePath()
        desired_position = np.array(list(map(lambda x: np.array([x[0] + (desired_follow_distance * math.sin(desired_follow_angle + x[2])), x[1] + (desired_follow_distance * math.cos(desired_follow_angle + x[2]))]), human_path)))
        robot = [robot_start]
        for algorithm_num in algorithm_nums:
            if algorithm_num == "spring":
                testAPF = MultiAgentAPF.MultiAgentAPF(desired_follow_distance=desired_follow_distance, desired_follow_angle=desired_follow_angle)
                error = [np.linalg.norm(robot_start - desired_position[0])]
                testAPF.updateRobot(robot_start) #Initial robot position
                for point in desired_position:
                    testAPF.updateEnvironment(point, [], .01)
                    robot_movement = testAPF.getRobotControlVelocity()
                    testAPF.updateRobot([testAPF.robot_position[0] + robot_movement[0], testAPF.robot_position[1] + robot_movement[1]])
                    robot.append(testAPF.robot_position)
                    error.append(np.linalg.norm(testAPF.robot_position - point))
                results.append(["spring", i, sum(error)/len(error), max(error)])
            elif algorithm_num == "astar":
                pass
        plt.plot([x[0] for x in human_path], [x[1] for x in human_path], '--o', label='human', c='green')
        plt.plot([x[0] for x in desired_position], [x[1] for x in desired_position], '--o', label='desired', c='red')
        # plt.plot([x[0] for x in desired_position], [x[1] for x in desired_position], '--o', label='robot')
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
    args = parser.parse_args()

    algorithm_nums = args.algorithms
    output_file = args.output
    num_trials = args.trials

    test_path_following_algorithms(algorithm_nums, output_file, num_trials)

if __name__ == "__main__":
    main()
