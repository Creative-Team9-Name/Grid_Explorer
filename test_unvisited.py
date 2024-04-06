import heapq
import math


y = 4
x = 6
start = (0,0)
position = (0,0)
red_count = 0
box_count = 0

grid = [[None for i in range(y)] for j in range(x)]
red_position = []
box_position = [(2, 0), (2,1)]
for pos_x, pos_y in box_position:
    grid[pos_x][pos_y] = 'box'

def nearest_unvisited_neighbor():
    unvisited = set()
    for i in range(x):
        for j in range(y):
            if grid[i][j] == None:
                unvisited.add((i, j))

    if len(unvisited) == 0:
        return None
    
    min_distance = float('inf')
    min_neighbor = None
    for (i, j) in unvisited:
        if (i, j) != position:
            distance = abs(i - position[0]) + abs(j - position[1])
            if distance < min_distance:
                min_distance = distance
                min_neighbor = (i, j)
    return min_neighbor


position = (0, 0)
grid[1][3] = 'B'
grid[2][3] = 'B'

while True:
    destination = nearest_unvisited_neighbor()
    if destination == None:
        break
    else:
        grid[destination[0]][destination[1]] = 'W'
        position = destination
        print("Destination:", destination)