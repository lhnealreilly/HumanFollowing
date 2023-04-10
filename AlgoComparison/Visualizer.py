#Credit to Zah on Stackoverflow for the dynamically updating matplotlib
import HumanPathGenerator
import matplotlib.pyplot as plt
import math
plt.ion()
class DynamicUpdate():
    #Suppose we know the x range
    min_x = -10
    max_x = 10
    min_y = -5
    max_y = 15

    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.human, = self.ax.plot([],[], 'o', label="human")
        self.spring, = self.ax.plot([],[], 'o', label="spring")
        self.astar, = self.ax.plot([],[], 'oy', label="astar")
        self.human_trace, = self.ax.plot([], [], 'b')
        self.robot_trace, = self.ax.plot([], [], 'r')
        self.astar_trace, = self.ax.plot([], [], 'y')

        self.occlusion1, = self.ax.plot([], [], 'g')
        self.occlusion2, = self.ax.plot([], [], 'g')
        self.occlusion3, = self.ax.plot([], [], 'g')
        self.occlusion4, = self.ax.plot([], [], 'g')
        #Autoscale on unknown axis and known lims on the other
        # self.ax.set_autoscaley_on(True)
        self.ax.set_xlim(self.min_x, self.max_x)
        self.ax.set_ylim(self.min_y, self.max_y)
        #Other stuff
        self.ax.legend()
        self.ax.grid()
        ...

    def on_running(self, human_path, robot_path, astar_path, occlusions):
        #Update data (with the new _and_ the old points)
        self.human.set_xdata(human_path[-1][0])
        self.human.set_ydata(human_path[-1][1])

        self.spring.set_xdata(robot_path[-1][0])
        self.spring.set_ydata(robot_path[-1][1])

        self.astar.set_xdata(astar_path[-1][0])
        self.astar.set_ydata(astar_path[-1][1])

        self.human_trace.set_xdata([x[0] for x in human_path])
        self.human_trace.set_ydata([x[1] for x in human_path])

        self.robot_trace.set_xdata([x[0] for x in robot_path])
        self.robot_trace.set_ydata([x[1] for x in robot_path])

        self.astar_trace.set_xdata([x[0] for x in astar_path])
        self.astar_trace.set_ydata([x[1] for x in astar_path])

        self.occlusion1.set_xdata([x[0] for x in occlusions[0]])
        self.occlusion1.set_ydata([x[1] for x in occlusions[0]])

        self.occlusion2.set_xdata([x[0] for x in occlusions[1]])
        self.occlusion2.set_ydata([x[1] for x in occlusions[1]])

        self.occlusion3.set_xdata([x[0] for x in occlusions[2]])
        self.occlusion3.set_ydata([x[1] for x in occlusions[2]])

        self.occlusion4.set_xdata([x[0] for x in occlusions[3]])
        self.occlusion4.set_ydata([x[1] for x in occlusions[3]])
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
        from AStarOpen import astar
        self.on_launch()

        follow_angle = math.pi/4

        testAPF = VirtualSpring.VirtualSpring(desired_follow_distance=2, desired_follow_angle=follow_angle)

        robot_vel_limit = 500#This is a simulated version of the physical velocity constraints of the robot in dist/time step
        
        time_step = 50

        h = HumanPathGenerator.HumanPathGenerator()
        human_path = h.generatePath(time_step=time_step, path_length=200)

        index = math.floor(len(human_path)/2)
        x, y = [human_path[index][0] + (2 * math.sin(follow_angle + human_path[index][2])), human_path[index][1] + (2 * math.cos(follow_angle + human_path[index][2]))]
        static_obstacles = [{'type': 'line', 'position': np.array([[x - 3, y], [x  - 4, y]])}, {'type': 'line', 'position': np.array([[x - 1, y], [x  +0, y]])}] #Static obstacles in the environment
        obstacle = static_obstacles[0]

        desired_position = np.array(list(map(lambda x: np.array([x[0] + (2 * math.sin(follow_angle + x[2])), x[1] + (2 * math.cos(follow_angle + x[2]))]), human_path)))

        arrow = plt.Arrow(*static_obstacles[0]['position'][0], *(static_obstacles[0]['position'][1] - static_obstacles[0]['position'][0]), label='wall', linewidth=0.01)
        plt.gca().add_patch(arrow)

        arrow = plt.Arrow(*static_obstacles[1]['position'][0], *(static_obstacles[1]['position'][1] - static_obstacles[1]['position'][0]), label='wall', linewidth=0.01)
        plt.gca().add_patch(arrow)
        plt.legend()
        plt.show(block=False) 
        
        testAPF.updateRobot([1, 1]) #Initial robot position
        testAPF.updateEnvironment(human_path[0], static_obstacles) #Initial environment

        xdata = list([0, 0])
        ydata = list([2, 0])

        robot_path = [[1, 1]]
        robot_path_astar = [[1, 1]]

        for x in range(1,200):
          robot_movement = testAPF.getRobotControlVelocity()
          length = math.sqrt(robot_movement[0] * robot_movement[0] + robot_movement[1] * robot_movement[1])
        #   robot_movement = list(map(lambda x: x*time_step, robot_movement))
          testAPF.updateEnvironment(human_path[x], static_obstacles) #Update environment
          testAPF.updateRobot([robot_path[-1][0] + robot_movement[0], robot_path[-1][1] + robot_movement[1]])
          found_path = astar(static_obstacles, tuple(robot_path_astar[-1]), [human_path[x][0] + (2 * math.sin(follow_angle + human_path[x][2])), human_path[x][1] + (2 * math.cos(follow_angle + human_path[x][2]))], limit=20)
          if len(found_path) > 0:
            robot_path_astar.extend(list(zip(found_path[0], found_path[1])))
          robot_path.append([robot_path[-1][0] + robot_movement[0], robot_path[-1][1] + robot_movement[1]])
          self.on_running(human_path[0:x], robot_path, robot_path_astar, testAPF.calc_occlusion_lines())
          time.sleep(time_step / 1000)
        return xdata, ydata

d = DynamicUpdate()
d()
