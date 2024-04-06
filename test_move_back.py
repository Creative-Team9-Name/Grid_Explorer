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
box_position = [(1, 1), (1,3)]
for pos_x, pos_y in box_position:
    grid[pos_x][pos_y] = 'B'


class Orientation(): # class to keep track of the orientation of the robot

    def __init__(self, dir = 0):
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] # N=0, E=1, S=2, W=3
        self.current = dir
        self.current_direction = self.directions[self.current]

    def turn_right(self):
        self.current = (self.current + 1) % 4
        self.current_direction = self.directions[self.current]

    def turn_left(self):
        self.current = (self.current - 1) % 4
        self.current_direction = self.directions[self.current]

    # for testing purposes
    def turn_around(self):
        self.turn_right()
        self.turn_right()

orientation = Orientation(0)
print("current:", orientation.current_direction)   

def shortest_path(destination):

    # djikstra to find the shortest path to starting position
    distance = []
    for i in range(x):
        row = []
        for j in range(y):
            row.append(float('inf'))
        distance.append(row)

    distance[position[0]][position[1]] = 0
    pq = [(0, position)]
    
    while pq:
        dist, node = heapq.heappop(pq)
        pos_x, pos_y = node
        if node == destination:
            break
        
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx = pos_x + dx
            ny = pos_y + dy
            if 0 <= nx < x and 0 <= ny < y and grid[nx][ny] != 'B':
                new_dist = dist + 1  # Cost of moving to a neighbor is 1
                if new_dist < distance[nx][ny]:
                    distance[nx][ny] = new_dist
                    heapq.heappush(pq, (new_dist, (nx, ny)))
    
    # Reconstruct the shortest path from start to target
    path = []
    while (pos_x, pos_y) != position:
        path.append((pos_x, pos_y))
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx = pos_x + dx
            ny = pos_y + dy
            if 0 <= nx < x and 0 <= ny < y and distance[nx][ny] == distance[pos_x][pos_y] - 1:
                pos_x, pos_y = nx, ny
                break
    path.reverse()
    return path

def goto_destination(destination): ### check if it's correct
    global position
    path = shortest_path(destination)
    print(path)
    for pos_x, pos_y in path:
        dir_x = pos_x - position[0]
        dir_y = pos_y - position[1]
        dir = (dir_x, dir_y)
        if dir != orientation.current_direction:
            print('dir:', dir, 'current:', orientation.current_direction)
            if dir == orientation.directions[(orientation.current + 1) % 4]:
                orientation.turn_right()
                print("turn right")
            elif dir == orientation.directions[(orientation.current - 1) % 4]:
                orientation.turn_left()
                print("turn left")
            else:
                orientation.turn_around()
                print("turn around")
        print("move straight")
        position = (position[0] + dir_x, position[1] + dir_y)
        print('position:', position)

position = (5, 1)
start = (0, 0)

goto_destination(start)
print(shortest_path)