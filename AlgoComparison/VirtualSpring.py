import math
import numpy as np

HUMAN_K_VALUE = .5 #The k-value of the virtual spring attached to the human
OBSTACLE_K_VALUE = .1 #The k-value of the virtual spring attached to the obstacles
# BOUNDARY_K_VALUE = .3 #The k-value if the robot gets within a boundary distance near obstacles
# BOUNDARY_DISTANCE = .2 #The distance in meters to switch to using the boubndary k-value

class VirtualSpring:
  def __init__(self, desired_follow_distance, desired_follow_angle=0) -> None:
    self.static_obstacles = []; # Position array of static obstacles
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

  def repulsive_force(self, point, circle_center, radius):
    distance = math.sqrt((point[0] - circle_center[0])**2 + (point[1] - circle_center[1])**2)
    angle = math.atan2(point[1] - circle_center[1], point[0] - circle_center[0])
    force_magnitude = 1 / (distance - radius)
    spring_k = OBSTACLE_K_VALUE
    # if distance - radius - BOUNDARY_DISTANCE < 0:
    #   spring_k = BOUNDARY_K_VALUE
    force_x = force_magnitude * math.cos(angle) * spring_k
    force_y = force_magnitude * math.sin(angle) * spring_k
    return (force_x, force_y)

  def updateEnvironment(self, human_position, obstacles):
    self.human_trajectory.append(human_position)
    self.human_angle = human_position[2]
    #Update obstacle array
    self.static_obstacles = obstacles
  
  def updateRobot(self, robot_position):
    self.robot_position = robot_position
    pass

  def getRobotControlVelocity(self):
    desired_position = [self.human_trajectory[-1][0] + (self.desired_follow_distance * math.sin(self.desired_follow_angle + self.human_angle)), self.human_trajectory[-1][1] + (self.desired_follow_distance * math.cos(self.desired_follow_angle + self.human_angle))]
    robot_velocity = np.array(list(map(lambda x: HUMAN_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position)))) #Default velocity with only the spring attached between the robot and human
    for obstacle in self.static_obstacles:
      obstacle_pos = obstacle[0:2]
      robot_velocity += np.array(list(self.repulsive_force(self.robot_position, obstacle_pos, obstacle[2])))
    return list(robot_velocity)




