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
# TODO: do not go out of the grid
# TODO: check black lines after movement to see if went too far


# the method:
#   1) find the yellow boxes
#       - 2 cases: yellow box on the vert1 line + no yellow box on the vert1 line
#   2) find the red cells 
#       - will already have 
#           coordinates of yellow boxes 
#           some of the cells checked 
#   3) go back to starting position

hub = PrimeHub()

color = ColorSensor('A') # change if necessary
distance_front = DistanceSensor('C') # change if necessary
distance_right = DistanceSensor('D') # change if necessary
motor_pair = MotorPair('E', 'F') #### change if necessary


y = 4
x = 6
start = (0,0)
position = (0,0)

red_count = 0
yellow_box_count = 0

grid = [[None for i in range(y)] for j in range(x)]
grid[0][0] = 'start'
grid[5][3] = 'start'



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
        self.motor = MotorPair(wheels[0], wheels[1])
    
    def move_straight(self):
        self.motor.move(23, unit='cm', steering=0, speed=100)
        position = position + self.direction.current_direction
        check_color()

    def turn_left(self):
        self.motor.move(0, unit='cm', steering=90, speed=50)
        self.direction.turn_left()
    
    def turn_right(self):
        self.motor.move(0, unit='cm', steering=-90, speed=50)
        self.direction.turn_right()

    def turn_around(self):
        self.turn_right()
        self.turn_right()


def check_color():   
        global red_count, grid

        cell_color = color.get_color()

        if (cell_color == 'red'):
            red_count  += 1
            grid[position.x][position.y] = 'R'
        else:
            grid[position.x][position.y] = 'Not R'

def check_horizontal(distance_sensor):
        global yellow_box_count

        cells_to_hor_edge = 5 - position.x

        if distance_sensor.get_distance() / 23 < cells_to_hor_edge:
                box_position = min(int(distance_sensor.get_distance() / 23) + 1, 5)
                grid[box_position][position.y] = 'B'
                yellow_box_count += 1

def check_vertical(distance_sensor):
        global yellow_box_count
        
        cells_to_vert_edge = 3 - position.x

        if distance_sensor.get_distance() / 23 < cells_to_vert_edge:
                box_position = min(int(distance_sensor.get_distance() / 23) + 1, 3)
                grid[box_position][position.y] = 'B'
                yellow_box_count += 1


                    # P A R T   1 ~~~~~~


def search_yellow_boxes(distance_front, distance_right, move):
        global position, yellow_box_count, red_count

        # at initial position ~> 
        check_vertical(distance_front)
        check_horizontal(distance_right)

        if yellow_box_count == 2: return grid    # case where we find 2 boxes on vert1 and hor1 lines
        elif yellow_box_count == 1:                 # case where we have one box in vert1 line
            if yellow_box_position[0].x == 0:
                  #get to position (1,0)
                  move.turn_right()
                  move.move_straight()
                  move.turn_left()

                  check_vertical(distance_front)

                  if yellow_box_count == 2: return grid # if case we find second box on vert2
                  else: 
                        while (distance_front.get_distance() / 23 - position.y > 1):
                            move.move_straight()
                            check_horizontal(distance_right)
                        if yellow_box_count == 2: return grid
                        else: 
                            move.turn_right()
                            check_horizontal(distance_front)
                            if yellow_box_count == 2: return grid
                            elif yellow_box_count == 1: return yellow_box_position.append((0,2))

        elif yellow_box_count == 0 or (yellow_box_count==1 and yellow_box_position[0].y == 0):
                while (distance_front.get_distance() / 23 - position.y > 1):
                            move.move_straight()
                            check_horizontal(distance_right)
                

                if yellow_box_count == 2: return grid
                elif yellow_box_count == 1: 
                    if yellow_box_position[0].y !=3:        # case when the top horizontal line is empty
                        while (distance_front.get_distance() / 23 - position.y > 1):
                            move.move_straight()
                            check_horizontal(distance_right)
                            return grid
                    else:           # case when it is on top horizontal line where one yellow box is hidden behind another
                        # move to (0,2) 
                        move.turn_around()
                        move.move_straight()

                        move.turn_left()

                        while (distance_front.get_distance() / 23 - position.y > 1):
                             move.move_straight()
                        move.turn_left()
                        move.move_straight()
                        move.turn_left()

                        check_horizontal(distance_front)

                        return grid


                
    

                    # P A R T   2 ~~~~~~


def search_red_cells(distance_front, distance_right, move):
        global red_count, yellow_box_count, position

        for i in range(x):
             for j in range (y):
                  if grid[i][j] == None:
                       move.move_to_position((i, j))  ##should implement this somehow + it should contain check_color funciton + while going around the yellow boxes
                       
        return grid
        

                    # P A R T   3 ~~~~~~ done kinda ~~~~~~~


def shortest_path_home():
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
    # path.append(position)
    path.reverse()
    return path

def return_home(move): ### check if it's correct
    path = shortest_path_home(move)
    for pos_x, pos_y in path:
        dir_x = pos_x - position[0]
        dir_y = pos_y - position[1]
        dir = (dir_x, dir_y)
        if dir != move.direction.current_direction:
            if move.direction.directions[(move.direction.current + 1 )% 4] == dir:
                move.turn_right()
            elif move.direction.directions[(move.direction.current - 1 )% 4] == dir:
                move.turn_left()
            else:
                move.turn_around()
        move.move_straight()


if __name__ =='__main__':
    initial_position()
    move = Move(('B', 'A'))

    while red_count != 2 and yellow_box_count != 2:
         
        # search for yellow boxes

        # search for red cells

        pass

    # return home
    return_home(move)
    print(grid)
    
