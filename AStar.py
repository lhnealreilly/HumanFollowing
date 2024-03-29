import heapq
import numpy as np


def h(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal, limit=None):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0),
                 (1, 1), (1, -1), (-1, 1), (-1, -1)]
    closed_list = set()
    came_from = {}
    g = {start: 0}
    f = {start: h(start, goal)}
    open_heap = []
    heapq.heappush(open_heap, (f[start], start))
    while len(open_heap) > 0:
        current = heapq.heappop(open_heap)[1]
        if current == goal or (limit is not None and len(closed_list) > limit):
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = (data+[start])[::-1]
            x_coords = []
            y_coords = []
            for i in (range(0,len(data))):
                x = data[i][0]
                y = data[i][1]
                x_coords.append(x)
                y_coords.append(y)
            return [x_coords, y_coords]
        closed_list.add(current)

        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = g[current] + h(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
            if neighbor in closed_list and tentative_g_score >= g.get(neighbor, 0):
                continue
            if tentative_g_score < g.get(neighbor, 0) or neighbor not in [i[1]for i in open_heap]:
                came_from[neighbor] = current
                g[neighbor] = tentative_g_score
                f[neighbor] = tentative_g_score + h(neighbor, goal)
                heapq.heappush(open_heap, (f[neighbor], neighbor))
    return []
