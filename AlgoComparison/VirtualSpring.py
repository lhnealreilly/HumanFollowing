import math
import numpy as np
from ObstacleHelpers import closestPointOnLine

HUMAN_K_VALUE = .5 #The k-value of the virtual spring attached to the human
HUMAN_BOUND_K_VALUE = .5 #The k-value of the virtual spring attached to the human
OBSTACLE_K_VALUE = .1 #The k-value of the virtual spring attached to the obstacles
OCCLUSION_CONSTANT = .5 #The k-value of the virtual spring attached to the occlusion lines
# BOUNDARY_K_VALUE = .3 #The k-value if the robot gets within a boundary distance near obstacles
# BOUNDARY_DISTANCE = .2 #The distance in meters to switch to using the boubndary k-value

class VirtualSpring:
  def __init__(self, desired_follow_distance, desired_follow_angle=0, vel_cap=.1) -> None:
    self.static_obstacles = []; # Position array of static obstacles
    self.human_trajectory = []; #Full trajectory array for followed agent
    self.human_velocity = []; #Stored human velocity vector
    self.human_angle = 0 #Last recorded human angle, currently set by the angle of the velocity

    self.robot_position = []; #Current robot position 
    self.robot_velocity = []; #Current robot velocity

    self.desired_follow_angle = desired_follow_angle #Desired angle to follow, 0 is directly in front
    self.desired_follow_distance = desired_follow_distance #Desired distance to follow the human
    self.vel_cap = vel_cap
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
    self.human_trajectory.append(np.array(human_position))
    self.human_angle = human_position[2]
    #Update obstacle array
    self.static_obstacles = obstacles
  
  def updateRobot(self, robot_position):
    self.robot_position = np.array(robot_position)
    pass
  
  #Creates a line of occlusion between the human and the closest point of each obstacles to the robot
  def calc_occlusion_lines(self):
    obstacles = self.static_obstacles
    human_position = self.human_trajectory[-1]

    lines = []

    for obstacle in obstacles:
      obstacle_type = obstacle['type']

      obstacle_pos = obstacle['position']
      if(obstacle_type == 'line'):
        diff1 = obstacle_pos[0] - human_position[0:2]
        diff2 = obstacle_pos[1] - human_position[0:2]
        lines.append(np.array([obstacle_pos[0], (obstacle_pos[0]) + diff1 * 100] ))
        lines.append(np.array([obstacle_pos[1], (obstacle_pos[1]) + diff2 * 100] ))
    return lines
        
  def dist(self, a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
  
  def getRobotControlVelocity(self, point=None):
    desired_position = point
    if point == None:
      desired_position = [self.human_trajectory[-1][0] + (self.desired_follow_distance * math.sin(self.desired_follow_angle + self.human_angle)), self.human_trajectory[-1][1] + (self.desired_follow_distance * math.cos(self.desired_follow_angle + self.human_angle))]

    robot_velocity = np.array([0, 0])
    #If the robot is within the desired range (currently set as +- 15 degrees and +- .25 meters) apply very little force towards the desired:
    if (self.desired_follow_angle - np.pi/12 < np.arctan2(*(self.robot_position - self.human_trajectory[-1][0:2])) - self.human_trajectory[-1][2] < self.desired_follow_angle + np.pi/12) and (self.desired_follow_distance - .25 < self.dist(self.human_trajectory[-1], self.robot_position) < self.desired_follow_distance + .25):
      robot_velocity = np.array(list(map(lambda x: HUMAN_BOUND_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position)))) #Default velocity with only the spring attached between the robot and human
    else:
      robot_velocity = np.array(list(map(lambda x: HUMAN_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position)))) #Default velocity with only the spring attached between the robot and human

    # if(math.sqrt((self.robot_position[0] - self.human_trajectory[-1][0])**2 + (self.robot_position[1] - self.human_trajectory[-1][1])**2) > self.desired_follow_distance * 1.1):
    #   robot_velocity += np.array(list(map(lambda x: HUMAN_K_VALUE/2 * (x[0] - x[1]), zip(self.human_trajectory[-1], self.robot_position)))) #Default velocity with only the spring attached between the robot and human
    
    for obstacle in self.static_obstacles:
      obstacle_type = obstacle['type']

      obstacle_pos = obstacle['position']

      if(obstacle_type == 'circle'):
        robot_velocity += np.array(list(self.repulsive_force(self.robot_position, obstacle_pos, obstacle_pos[2])))
      elif(obstacle_type == 'line'):
        robot_velocity += np.array(list(self.repulsive_force(self.robot_position, closestPointOnLine(obstacle_pos[0], obstacle_pos[1], self.robot_position), 0)))
    
    for occlusion_line in self.calc_occlusion_lines():
      robot_velocity += (occlusion_line[1] - occlusion_line[0]) / np.linalg.norm(occlusion_line) * -1 / self.dist(occlusion_line[0], self.robot_position) * OCCLUSION_CONSTANT


    robot_velocity = robot_velocity if np.linalg.norm(robot_velocity) < self.vel_cap else robot_velocity / np.linalg.norm(robot_velocity) * self.vel_cap
    return list(robot_velocity)




