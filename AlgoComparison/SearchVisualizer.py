import numpy as np  
import matplotlib
import matplotlib.pyplot as plt
import time
from AStarOpen import astar

m = np.array([
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
[0, 1, 1, 1, 1, 1, 1, 0, 0, 1],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

obstacles = []

for i in range(len(m)):
    for j in range(len(m[i])):
        if m[i][j] == 1:
            obstacles.append(tuple([i, j, .5 * 1.42])) #Circle bounding area

grid = m

robot = tuple([2, 3])

human = tuple([4, 5])

humanGoal = tuple([9, 9])

humanPath = astar(obstacles, human, humanGoal)

# robotPath = astar(obstacles, robot, human)

# plot map and path

fig, ax = plt.subplots(figsize=(12,12))

ax.imshow(grid, cmap=plt.cm.Paired)

ax.scatter(humanGoal[1],humanGoal[0], marker = "s", color = "red", s = 200)

fig.canvas.draw()
plt.show(block=False)


for value in range(len(humanPath[1])-1):
    human = (humanPath[0][value],humanPath[1][value])
    ax.plot(humanPath[1][value:value+2],humanPath[0][value:value+2], color = "black")
    # if abs(robot[0] - human[0]) + abs(robot[1] - human[1]) >= 2 and len(robotPath)==2: 
    #     robot = (robotPath[0][1],robotPath[1][1])
    #     ax.plot(robotPath[1][0:min(len(robotPath[1]), 2)],robotPath[0][0:min(len(robotPath[1]), 2)], color = "green")
    # robotPath = astar(grid, robot, human, 10)
    
    ax.scatter(robot[1],robot[0], marker = "*", color = "yellow", s = 200)
    ax.scatter(human[1],human[0], marker = "x", color = "red", s = 200)
    ax.set_xticks(np.arange(0, 20, 1))
    ax.set_yticks(np.arange(0, 20, 1))  

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(.1)






