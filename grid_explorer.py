from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
import heapq
import math

### every time the robot moves and red_count < check red cell

# TODO: red cell identificator - calculate path to move around the map (avoid yellow box) 
# TODO: movement to search for yellow box - take in mind all the possible special cases
# TODO: going to starting position ----- ANGELA
# TODO: interaction with robot usign bluetooth to give stating position ---- ANGELA (with buttons)

### security check
# TODO: look alway foward while moving to see if we are going against something
# TODO: do not go out of the grid
# TODO: check black lines after movement to see if went too far

hub = PrimeHub()

color = ColorSensor('E')
distance = DistanceSensor('F')
motor_pair = MotorPair('B', 'A') ### decide whether to keep or not, based on the movement class


y = 4
x = 6
start = (0,0)
position = (0,0)
red_count = 0
box_count = 0
red_position = []
box_position = []
grid = [[None for i in range(y)] for j in range(x)]

# graph method
# eliminate from graph the cells that contain box
# use dfs to find the best path back home
# traveling-salesman algorithm to explore the grid until we find red cell
# find yellow boxes forst al all!!
#jjbsdvoavp


# wait for starting position
def initial_position():
    # wait for button to be pressed
    while True:
        if hub.left_button.is_pressed():
            start = (0,0)
            position = (0,0)
            break
        if hub.right_button.is_pressed():
            start = (5,3)
            position = (5,3)
            Orientation
            break

class Orientation(): # maybe not needed

    def __init__(self, orientation = 1):
        self.directions = [(1, 0), (0, 1), (-1, 0), (0, -1)] # N=0, E=1, S=2, W=3
        self.current = orientation
        self.current_direction = self.directions[self.current]
    
    def turn_right(self):
        self.current = (self.current + 1) % 4
        self.current_direction = self.directions[self.current]
    
    def turn_left(self):
        self.current = (self.current - 1) % 4
        self.current_direction = self.directions[self.current]

class Move():

    def __init__(self, wheels): # front_wheels = (right, left)
        if position == (0,0):
            self.direction = Orientation(0)
        else:
            self.direction = Orientation(2)
        self.motor = MotorPair(wheels[0], wheels[1])
    
    def move_straight(self):
        self.motor.move(23, unit='cm', steering=0, speed=100)
        position = position + self.direction.current_direction

    def turn_left(self):
        self.motor.move(0, unit='cm', steering=90, speed=50)
        self.direction.turn_left()
    
    def turn_right(self):
        self.motor.move(0, unit='cm', steering=-90, speed=50)
        self.direction.turn_right()

# check color function that checks color when it's on a new cell and save value in matrix
# while red_count = 2 and box_count = 2:

def check_color(grid, red_count, red_position): ## parameters are not needed
        cell_color = color.get_color()
        grid[position.x][position.y] = cell_color ## check if it's correct
        if (cell_color == 'red'):
            red_count+=1
            red_position.append((position.x, position.y))

def check_box(orientation):
        box_distance = distance.get_distance(short_range=False)
        box_position = int(box_distance / 23) + 1  
        # anged from ~> 
        # x_position = box_position
        # y_position = '?'
        x_position = position.x + orientation.current_direction[0] * box_position
        y_position = position.y + orientation.current_direction[1] * box_position
        return x_position, y_position

def shortest_path_home():  ### tested, correct!
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
        if node == start:
            break
        
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx = pos_x + dx
            ny = pos_y + dy
            if 0 <= nx < x and 0 <= ny < y and grid[nx][ny] != 'box':
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
    path.append(position)
    path.reverse()
    return path

def return_home(move): ### check if it's correct
    path = shortest_path_home(move)
    for pos_x, pos_y in path:
        if pos_x == position.x:
            if pos_y > position.y:
                move.move_straight()
            else:
                move.turn_left()
                move.move_straight()
        elif pos_y == position.y:
            if pos_x > position.x:
                move.turn_right()
                move.move_straight()
            else:
                move.turn_left()
                move.turn_left()
                move.move_straight()


def sec_checks():
        # Stop if grid edge, no extra cells of length 23 and more
        if distance.get_distance() < 23:
            motor_pair.stop()
        
        if color.get_color() == 'black':
            update_position()

def update_position(orientation):
        global position
        position = (position.x + orientation.current_direction[0], position.y + orientation.current_direction[1])


if __name__ =='__main__':
    initial_position()
    move = Move(('B', 'A')) #TODO: check if the ports are correct

    
    print(grid)
    
