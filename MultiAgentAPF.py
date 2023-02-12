import math
import numpy as np

CONFIDENCE_WEIGHTING = 2 #The multiplier on the most recent error when calculating confidence
TURN_WEIGHTING = .2 #The percentage of average turn velocity that is the most recent turn velocity
HUMAN_K_VALUE = .6 #The k-value of the virtual spring attached to the human
OBSTACLE_K_VALUE = -.1 #The k-value of the virtual spring attached to the obstacles

class MultiAgentAPF:
  def __init__(self, desired_follow_distance, desired_follow_angle=0) -> None:
    #State to keep track of the static and dynamic objects in the environment
    self.static_obstacles = []; # Position array of static obstacles
    self.dynamic_obstacles = []; # Position, average velocity, and confidence rating for dynamic obstacles

    self.human_trajectory = []; #Full trajectory array for followed agent
    self.human_velocity = []; #Stored human velocity vector
    self.human_turning_velocity = 0 #Stored average turning velocity weighted heavily towards recent motion
    self.human_prediction = []; #Velocity vector of last human prediction to be compared to actual movement
    self.human_confidence = 0; #Average accuracy of prediction for human trajectory weighted toward recent accuracy
    self.human_angle = 0 #Last recorded human angle, currently set by the angle of the velocity

    self.robot_position = []; #Current robot position 
    self.robot_velocity = []; #Current robot velocity

    self.desired_follow_angle = desired_follow_angle #Desired angle to follow, 0 is directly behind
    self.desired_follow_distance = desired_follow_distance #Desired distance to follow the human
    pass

  def predictHumanVelocity(self, human_velocity, human_turning_velocity):
    theta = np.radians(human_turning_velocity)
    c, s = np.cos(theta), np.sin(theta)
    rotation_matrix = np.array(((c,-s), (s, c)))
    return np.dot(rotation_matrix, human_velocity)

  def turnAngle(self, vel1, vel2):
    if(np.linalg.norm(vel1) == 0 or np.linalg.norm(vel2) == 0):
      return 0
    unit_vector_1 = vel1 / np.linalg.norm(vel1)
    unit_vector_2 = vel2 / np.linalg.norm(vel2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    return np.arccos(dot_product)

  def updateEnvironment(self, human_position, obstacle_positions, delta):
    #Update obstacle array
    self.static_obstacles = obstacle_positions

    delta = max(delta, 0.000000001)
    
    self.human_trajectory.append(human_position)

    if(len(self.human_trajectory) > 1):
      old_vel = self.human_velocity
      self.human_velocity = [(x[0] - x[1]) / delta for x in zip(self.human_trajectory[-1], self.human_trajectory[-2])]
    if(len(self.human_trajectory) > 2):
      self.human_turning_velocity = self.human_turning_velocity*(1-TURN_WEIGHTING) + self.turnAngle(old_vel, self.human_velocity)*TURN_WEIGHTING #Update average turn velocity
      self.human_angle = self.turnAngle([0, 1], self.human_velocity)
      self.human_prediction = self.predictHumanVelocity(self.human_velocity, self.human_turning_velocity)
      pred_location = [sum(x) for x in zip(self.human_trajectory[-2], map(lambda x : x * delta, self.human_prediction))] #Find what the predicted location was after this delta
      confidence = 1 / math.dist(pred_location, human_position) if math.dist(pred_location, human_position) > 0 else 1 #Confidence is 1/error where error is the disance between the prediction and the true
      self.human_confidence = (self.human_confidence * len(self.human_trajectory) + confidence * CONFIDENCE_WEIGHTING) / (len(self.human_trajectory) + CONFIDENCE_WEIGHTING) #Update running confidence for human prediction
    
    
  
  def updateRobot(self, robot_position):
    self.robot_position = robot_position
    pass

  def getRobotControlVelocity(self):
    desired_position = [self.human_trajectory[-1][0] + (self.desired_follow_distance * math.sin(self.desired_follow_angle + self.human_angle)), self.human_trajectory[-1][0] + (self.desired_follow_distance * math.cos(self.desired_follow_angle + self.human_angle))]

    robot_velocity = map(lambda x: HUMAN_K_VALUE * (x[0] - x[1]), zip(desired_position, self.robot_position)) #Default velocity with only the spring attached between the robot and human

    for obstacle in self.static_obstacles:
      robot_velocity = map(lambda x: OBSTACLE_K_VALUE * (x[0] - x[1]) + x[2], zip(obstacle, self.robot_position, robot_velocity))
    
    return list(robot_velocity)




