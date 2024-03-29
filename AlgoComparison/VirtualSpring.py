import math
import numpy as np
from ObstacleHelpers import closestPointOnLine
import configparser

class VirtualSpring:
  def __init__(self, desired_follow_distance, config_file="", desired_distance_range=[.25, .25], desired_angle_range=[np.pi/12, np.pi/12], desired_follow_angle=0, vel_cap=.1) -> None:
    self.static_obstacles = []; # Position array of static obstacles
    self.human_trajectory = []; #Full trajectory array for followed agent
    self.human_velocity = []; #Stored human velocity vector
    self.human_angle = 0 #Last recorded human angle, currently set by the angle of the velocity

    self.robot_position = []; #Current robot position 
    self.robot_velocity = []; #Current robot velocity

    self.desired_follow_angle = desired_follow_angle #Desired angle to follow, 0 is directly in front
    self.desired_follow_distance = desired_follow_distance #Desired distance to follow the human
    self.desired_angle_range = desired_angle_range
    self.desired_distance_range = desired_distance_range
    self.vel_cap = vel_cap

    if(config_file != ""):
      Config = configparser.ConfigParser()
      try:
        Config.read(config_file)
        self.HUMAN_K_VALUE = float(Config['SinkValues']['OutsideDesired']) #The k-value of the virtual spring attached to the human
        self.HUMAN_BOUND_K_VALUE = float(Config['SinkValues']['Desired']) #The k-value of the virtual spring attached to the human
        self.OBSTACLE_K_VALUE = float(Config['SourceValues']['Obstacles']) #The k-value of the virtual spring attached to the obstacles
        self.OCCLUSION_CONSTANT = float(Config['SourceValues']['Occlusion']) #The k-value of the virtual spring attached to the occlusion lines
        self.HUMAN_REPULSIVE = float(Config['SourceValues']['Human']) #The k-value of the source the pushes the robot away from the human
      except:
        print("Invalid Configuration File")
    else:
      self.HUMAN_K_VALUE = .5 #The k-value of the virtual spring attached to the human
      self.HUMAN_BOUND_K_VALUE = .1 #The k-value of the virtual spring attached to the human
      self.OBSTACLE_K_VALUE = .1 #The k-value of the virtual spring attached to the obstacles
      self.OCCLUSION_CONSTANT = .4 #The k-value of the virtual spring attached to the occlusion lines
      self.HUMAN_REPULSIVE = .4 #The k-value of the source the pushes the robot away from the human

    pass

  #Predicts a location for a human time_step into the future using a dynamics model on the previous human locations
  def human_prediction(self, time_step):
    #TODO
    pass


  def repulsive_force(self, point, circle_center, radius, spring_k):
    distance = math.sqrt((point[0] - circle_center[0])**2 + (point[1] - circle_center[1])**2)
    angle = math.atan2(point[1] - circle_center[1], point[0] - circle_center[0])
    force_magnitude = 1 / abs(distance - radius)
    force_x = force_magnitude * math.cos(angle) * spring_k
    force_y = force_magnitude * math.sin(angle) * spring_k
    return (force_x, force_y)

  def updateEnvironment(self, human_position, obstacles):
    self.human_trajectory.append(np.array(human_position))
    if(len(self.human_trajectory) > 50):
      self.human_trajectory = self.human_trajectory[-50:]
    self.human_angle = human_position[2]
    #Update obstacle array
    self.static_obstacles = obstacles
  
  def updateRobot(self, robot_position):
    self.robot_position = np.array(robot_position)
    pass
  
  #Creates a line of occlusion between the human and the closest point of each obstacles to the robot
  def calc_occlusion_lines(self, human_position):
    obstacles = self.static_obstacles

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
    if (self.desired_follow_angle - self.desired_angle_range[0] < np.arctan2(*(self.robot_position - self.human_trajectory[-1][0:2])) - self.human_trajectory[-1][2] < self.desired_follow_angle + self.desired_angle_range[1]) and (self.desired_follow_distance - self.desired_distance_range[0] < self.dist(self.human_trajectory[-1], self.robot_position) < self.desired_follow_distance + self.desired_distance_range[1]):
      robot_velocity = np.array(list(map(lambda x: self.HUMAN_BOUND_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position)))) #Default velocity with only the spring attached between the robot and human
    else:
      robot_attractive = np.array(list(map(lambda x: self.HUMAN_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position)))) #Default velocity with only the spring attached between the robot and human
      if(np.linalg.norm(robot_attractive) > 1):
        robot_attractive /= np.linalg.norm(robot_attractive)
      robot_velocity = robot_attractive

    #Human repulsive force so the robot doesn't run into the human
    if (self.dist(self.robot_position, self.human_trajectory[-1]) < self.desired_follow_distance / 2):
      robot_velocity += np.array(list(self.repulsive_force(self.robot_position, self.human_trajectory[-1], .3, self.HUMAN_REPULSIVE)))

    for obstacle in self.static_obstacles:
      obstacle_type = obstacle['type']

      obstacle_pos = obstacle['position']

      if(obstacle_type == 'circle'):
        robot_velocity += np.array(list(self.repulsive_force(self.robot_position, obstacle_pos, obstacle_pos[2], self.OBSTACLE_K_VALUE)))
      elif(obstacle_type == 'line'):
        closest_point_on_line = closestPointOnLine(obstacle_pos[0], obstacle_pos[1], self.robot_position)
        robot_velocity += np.array(list(self.repulsive_force(self.robot_position, closest_point_on_line, 0, self.OBSTACLE_K_VALUE)))

        #Create an occlusion force in the direction between the object and the human
        if self.dist(closest_point_on_line, self.robot_position) < 1:
          diff1 = closest_point_on_line - self.human_trajectory[-1][0:2]
          robot_velocity += ((((closest_point_on_line) + diff1 * 100) - closest_point_on_line) / np.linalg.norm(np.array([closest_point_on_line, (closest_point_on_line) + diff1 * 100] )) * -1 / self.dist(closest_point_on_line, self.robot_position) * self.OCCLUSION_CONSTANT)
    
    # for occlusion_line in self.calc_occlusion_lines(self.human_trajectory[-1]):
    #   #If near an occluding obstacle
    #   if(self.dist(occlusion_line[0], self.robot_position) < 1):
    #     robot_velocity += (occlusion_line[1] - occlusion_line[0]) / np.linalg.norm(occlusion_line) * -1 / self.dist(occlusion_line[0], self.robot_position) * self.OCCLUSION_CONSTANT


    robot_velocity = robot_velocity if np.linalg.norm(robot_velocity) < self.vel_cap else robot_velocity / np.linalg.norm(robot_velocity) * self.vel_cap
    return list(robot_velocity)




