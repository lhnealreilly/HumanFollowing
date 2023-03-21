#Credit to Zah on Stackoverflow for the dynamically updating matplotlib
import HumanPathGenerator
import matplotlib.pyplot as plt
import math
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
        import VirtualSpring
        self.on_launch()

        follow_angle = math.pi

        testAPF = VirtualSpring.VirtualSpring(desired_follow_distance=2, desired_follow_angle=follow_angle)

        robot_vel_limit = 500#This is a simulated version of the physical velocity constraints of the robot in dist/time step
        # static_obstacles = [[1, 1], [.2, 3]] #Static obstacles in the environment
        static_obstacles = []
        time_step = 100

        h = HumanPathGenerator.HumanPathGenerator()
        human_path = h.generatePath(time_step=time_step)

        desired_position = np.array(list(map(lambda x: np.array([x[0] + (2 * math.sin(follow_angle + x[2])), x[1] + (2 * math.cos(follow_angle + x[2]))]), human_path)))
        plt.plot([x[0] for x in desired_position], [x[1] for x in desired_position], label='desired', c='green', linewidth=1)
        plt.show(block=False) 

        testAPF.updateRobot([0, 0]) #Initial robot position
        testAPF.updateEnvironment(human_path[0], static_obstacles) #Initial environment

        xdata = list([0, 0])
        xdata.extend([p[0] for p in static_obstacles])
        ydata = list([2, 0])
        ydata.extend([p[0] for p in static_obstacles])

        for x in range(0,100):
          robot_movement = testAPF.getRobotControlVelocity()
          length = math.sqrt(robot_movement[0] * robot_movement[0] + robot_movement[1] * robot_movement[1])
        #   robot_movement = list(map(lambda x: x*time_step, robot_movement))
          testAPF.updateEnvironment(human_path[x], static_obstacles) #Update environment
          testAPF.updateRobot([xdata[1] + robot_movement[0], ydata[1] + robot_movement[1]])
          xdata[1] = xdata[1] + robot_movement[0]
          ydata[1] = ydata[1] + robot_movement[1]
          xdata[0] = human_path[x][0]
          ydata[0] = human_path[x][1]
          self.on_running(xdata, ydata)
          time.sleep(time_step / 1000)
        return xdata, ydata

d = DynamicUpdate()
d()
