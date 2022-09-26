import numpy as np
import matplotlib.pyplot as plt
import time
from AStar import astar
from mazelib import Maze
from mazelib.generate.DungeonRooms import DungeonRooms

m = Maze()
m.generator = DungeonRooms(10, 10)
m.generate()
m.generate_entrances(start_outer=False, end_outer=False)

grid = m.grid

robot = m.start

human = m.start

humanGoal = m.end

print(human)
humanPath = astar(grid, human, humanGoal)


robotPath = astar(grid, robot, human)

# plot map and path

fig, ax = plt.subplots(figsize=(12,12))

ax.imshow(grid, cmap=plt.cm.Paired)

ax.scatter(humanGoal[1],humanGoal[0], marker = "s", color = "red", s = 200)

fig.canvas.draw()
plt.show(block=False)


for value in range(len(humanPath[1])-1):
    human = (humanPath[0][value],humanPath[1][value])
    ax.plot(humanPath[1][value:value+2],humanPath[0][value:value+2], color = "black")
    if abs(robot[0] - human[0]) + abs(robot[1] - human[1]) >= 2 and len(robotPath)==2: 
        robot = (robotPath[0][1],robotPath[1][1])
        ax.plot(robotPath[1][0:min(len(robotPath[1]), 2)],robotPath[0][0:min(len(robotPath[1]), 2)], color = "green")
    robotPath = astar(grid, robot, human)
    
    ax.scatter(robot[1],robot[0], marker = "*", color = "yellow", s = 200)
    ax.scatter(human[1],human[0], marker = "x", color = "red", s = 200)
    ax.set_xticks(np.arange(0, 20, 1))
    ax.set_yticks(np.arange(0, 20, 1))  

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(.1)






