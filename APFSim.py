#Credit to Zah on Stackoverflow for the dynamically updating matplotlib

import matplotlib.pyplot as plt
plt.ion()
class DynamicUpdate():
    #Suppose we know the x range
    min_x = -5
    max_x = 10
    min_y = -5
    max_y = 10

    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], 'o')
        #Autoscale on unknown axis and known lims on the other
        # self.ax.set_autoscaley_on(True)
        self.ax.set_xlim(self.min_x, self.max_x)
        self.ax.set_ylim(self.min_y, self.max_y)
        #Other stuff
        self.ax.grid()
        ...

    def on_running(self, xdata, ydata):
        #Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    #Example
    def __call__(self):
        import numpy as np
        import time
        import math
        import MultiAgentAPF
        self.on_launch()
        testAPF = MultiAgentAPF.MultiAgentAPF(desired_follow_distance=1)

        robot_vel_limit = .2 #This is a simulated version of the physical velocity constraints of the robot in dist/time step
        static_obstacles = [[1, 1]] #Static obstacles in the environment
        time_step = 1

        testAPF.updateRobot([0, 0]) #Initial robot position
        testAPF.updateEnvironment([0, 1], static_obstacles, 0) #Initial environment

        xdata = list([0, 0, 0])
        xdata.extend([p[0] for p in static_obstacles])
        ydata = list([2, 0, 1])
        ydata.extend([p[0] for p in static_obstacles])

        for x in np.arange(0,100,time_step):
          robot_movement = testAPF.getRobotControlVelocity()
          length = math.sqrt(robot_movement[0] * robot_movement[0] + robot_movement[1] * robot_movement[1])
          robot_movement = list(map(lambda x: x/length * (robot_vel_limit), robot_movement))
          testAPF.updateEnvironment([xdata[0], ydata[0]], static_obstacles, time_step) #Update environment
          # xdata[0] = xdata[0]
          # ydata[0]
          self.on_running(xdata, ydata)
          testAPF.updateRobot([xdata[1] + robot_movement[0], ydata[1] + robot_movement[1]])
          xdata[1] = xdata[1] + robot_movement[0]
          ydata[1] = ydata[1] + robot_movement[1]
          time.sleep(1)
        return xdata, ydata

d = DynamicUpdate()
d()
