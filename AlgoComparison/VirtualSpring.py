import math
import numpy as np

HUMAN_K_VALUE = .6 #The k-value of the virtual spring attached to the human

class VirtualSpring:
  def __init__(self, desired_follow_distance, desired_follow_angle=0) -> None:

    self.human_trajectory = []; #Full trajectory array for followed agent
    self.human_velocity = []; #Stored human velocity vector
    self.human_angle = 0 #Last recorded human angle, currently set by the angle of the velocity

    self.robot_position = []; #Current robot position 
    self.robot_velocity = []; #Current robot velocity

    self.desired_follow_angle = desired_follow_angle #Desired angle to follow, 0 is directly in front
    self.desired_follow_distance = desired_follow_distance #Desired distance to follow the human
    pass

  def turnAngle(self, vel1, vel2):
    if(np.linalg.norm(vel1) == 0 or np.linalg.norm(vel2) == 0):
      return 0
    unit_vector_1 = vel1 / np.linalg.norm(vel1)
    unit_vector_2 = vel2 / np.linalg.norm(vel2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    return np.arccos(dot_product)

  def updateEnvironment(self, human_position, obstacle_positions):
    self.human_trajectory.append(human_position)
    self.human_angle = human_position[2]
  
  def updateRobot(self, robot_position):
    self.robot_position = robot_position
    pass

  def getRobotControlVelocity(self):
    desired_position = [self.human_trajectory[-1][0] + (self.desired_follow_distance * math.sin(self.desired_follow_angle + self.human_angle)), self.human_trajectory[-1][1] + (self.desired_follow_distance * math.cos(self.desired_follow_angle + self.human_angle))]
    robot_velocity = map(lambda x: HUMAN_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position)) #Default velocity with only the spring attached between the robot and human

    return list(robot_velocity)




