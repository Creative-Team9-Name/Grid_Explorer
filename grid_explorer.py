from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
import heapq


# the method:
#1) find the yellow boxes
#    - 2 cases: yellow box on the vert1 line + no yellow box on the vert1 line
#2) find the red cells
#    - will already have
#        coordinates of yellow boxes
#        some of the cells checked
#3) go back to starting position

hub = PrimeHub()

color = ColorSensor('A')
distance_front = DistanceSensor('C')
distance_right = DistanceSensor('D')

y = 4
x = 6
start = (0,0)
position = (0,0)

red_count = 0
box_count = 0

grid = [[None for i in range(y)] for j in range(x)]
grid[0][0] = 'W'
grid[5][3] = 'W'


# wait for starting position
def initial_position():
    print('hello')
    hub.light_matrix.write('9')
    # wait for button to be pressed
    while True:
        if hub.left_button.is_pressed():
            start = (0,0)
            position = (0,0)
            return
        if hub.right_button.is_pressed():
            start = (5,3)
            position = (5,3)
            return


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


class Move(): # class to move the robot

    def __init__(self, wheels): # front_wheels = (right, left)
        self.motor = MotorPair(wheels[0], wheels[1])
        if start == (0, 0):
            self.direction = Orientation()
        else:
            self.direction = Orientation(2)

    def move_straight(self):
        global position
        self.motor.move_tank(amount = 23.5 , unit='cm', left_speed=-50, right_speed=-50)
        self.pos_x = position[0] + self.direction.current_direction[0]
        self.pos_y = position[1] + self.direction.current_direction[1]
        position = (self.pos_x, self.pos_y)
        time.sleep(1)

    def turn_left(self):
        hub.motion_sensor.reset_yaw_angle()
        while hub.motion_sensor.get_yaw_angle() > -87:
            self.motor.start_tank(left_speed=-50, right_speed=50)
            time.sleep(0.1)
            self.motor.stop()
        # angle = hub.motion_sensor.get_yaw_angle()
        # print('Angle:', angle)
        self.direction.turn_left()

    def turn_right(self):
        hub.motion_sensor.reset_yaw_angle()
        while hub.motion_sensor.get_yaw_angle() < 87:
            self.motor.start_tank(left_speed=50, right_speed=-50)
            time.sleep(0.1)
            self.motor.stop()
        # angle = hub.motion_sensor.get_yaw_angle()
        # print('Angle:', angle)
        self.direction.turn_right()

    def turn_around(self):
        self.turn_right()
        self.turn_right()


def check_color(): # check the color of the cell
    cell_color = None
    i = 0
    while cell_color == None and i < 10:
        cell_color = color.get_color()
        if (cell_color == 'red'):
            red_count+= 1
            grid[position[0]][position[1]] = 'R'
        else:
            grid[position[0]][position[1]] = 'W'
        time.sleep(0.05)
        i += 1

def check_horizontal(distance_sensor): # check if there is a yellow box on the horizontal line
    global box_count, grid, position
    distance = distance_sensor.get_distance_cm()
    if distance != None:
        box_position = int(distance / 23) + position[0] +1
        if box_position <= 5:
            grid[box_position][position[1]] = 'B'
            box_count += 1

def check_vertical(distance_sensor): # check if there is a yellow box on the vertical line
    global box_count, grid, position
    distance = distance_sensor.get_distance_cm()

    if distance != None:
        box_position = int(distance / 23) + position[1] +1
        if box_position <= 3:
            grid[position[0]][box_position] = 'B'
            box_count += 1
'''
def check_horizontal(distance_sensor): # check if there is a yellow box on the horizontal line
    global box_count, grid, position
    distance = None
    i = 0
    print("check horizontal")
    while distance == None and i < 5:
        print(i)
        distance = distance_sensor.get_distance_cm()
        if distance != None:
            box_position = int(distance / 23) + position[0] +1
            if box_position <= 5:
                grid[box_position][position[1]] = 'B'
                box_count += 1
        i += 1

def check_vertical(distance_sensor): # check if there is a yellow box on the vertical line
    global box_count, grid, position
    distance = None
    i = 0
    print("check vertical")
    if distance == None and i < 5:
        print(i)
        distance = distance_sensor.get_distance_cm()
        if distance != None:
            box_position = int(distance / 23) + position[1] +1
            if box_position <= 3:
                grid[position[0]][box_position] = 'B'
                box_count += 1
        i += 1

'''

def search_yellow_boxes(distance_front, distance_right, move):
    global position, box_count, red_count

    # at initial position ~>
    check_vertical(distance_front)
    time.sleep(1)
    check_horizontal(distance_right)
    time.sleep(1)

    if box_count < 2:    # case where we find 2 boxes on vert1 and hor1 lines
        if any(grid[start[0]][j] == 'B' for j in range(y)):
            move.turn_right()
            move.move_straight()
            check_color()
            move.turn_left()
            check_vertical(distance_front)

            if box_count < 2:
                for i in range(3):
                    move.move_straight()
                    check_horizontal(distance_right)
                if box_count < 2:
                    move.turn_left()
                    check_horizontal(distance_front)
                    if box_count < 2:
                        grid[start[0]][abs (start[1]-2)] = 'B'

        else:
            for i in range(3):
                move.move_straight()
                check_color()
                check_horizontal(distance_right)


            if box_count < 2:
                if any(grid[j][abs(start[1]-3)] == 'B' for j in range(5)): # case when it is on top horizontal line where one yellow box is hidden behind another
                    move.turn_around()
                    move.move_straight()
                    check_color()
                    move.turn_left()

                    for i in range (5):
                        move.move_straight()
                        check_color()
                    move.turn_left()
                    check_horizontal(distance_front)
                    if box_count < 2:
                        move.move_straight()
                        check_color()
                        move.turn_left()
                        check_horizontal(distance_front)
                else:        # when some other horizontal line and the box hidden behind it, we check vertical lines while moving horizontally
                    for i in range (5):
                        move.turn_right()
                        move.move_straight()
                        check_color()
                        check_vertical(distance_right)

    return grid


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
                new_dist = dist + 1# Cost of moving to a neighbor is 1
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


def goto_destination(move, destination): ### check if it's correct
    global position
    path = shortest_path(destination)
    print(path)
    for pos_x, pos_y in path:
        dir_x = pos_x - position[0]
        dir_y = pos_y - position[1]
        dir = (dir_x, dir_y)
        if dir != move.direction.current_direction:
            print('dir:', dir, 'current:', move.direction.current_direction)
            if dir == move.direction.directions[(move.direction.current + 1) % 4]:
                move.turn_right()
                # print("turn right")
            elif dir == move.direction.directions[(move.direction.current - 1) % 4]:
                move.turn_left()
                # print("turn left")
            else:
                move.turn_around()
                # print("turn around")
        move.move_straight()
        # print("move straight")
        print("position ", position)


def search_red_cells(move): ### check if it's correct
    while(red_count < 2):
        destination = nearest_unvisited_neighbor()
        if destination == None:
            print("Error: no unvisited neighbor!")
            return
        else:
            print("Nearest neighbor: ", destination)
            goto_destination(move, destination)
            check_color()


################################### main ###################################

initial_position()
move = Move(('E', 'F'))

print("find yellow boxes")
search_yellow_boxes(distance_front, distance_right, move)
# grid[1][1] = 'B'
# grid[1][3] = 'B'

print("search red cells")
search_red_cells(move)
# position = (4,3)

print("Go home")
goto_destination(move, start)

for i in range(x):
    for j in range(y):
        if grid[i][j] == 'B':
            print("(" + str(i) + "," + str(j)+ "," + "B" + ")")
        elif grid[i][j] == 'R':
            print("(" + str(i) + "," + str(j) + "," + "R" + ")")
        else:
            print("(" + str(i) + "," + str(j) + "," + "W" + ")")


############################################################################