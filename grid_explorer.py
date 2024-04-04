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


#### sensors, motors and ports
# front distance sensor = C
# right distance sensor = D
# Color sensor = A
# right motor = E
# left motor = F

# the method:
#   1) find the yellow boxes
#       - 2 cases: yellow box on the vert1 line + no yellow box on the vert1 line
#   2) find the red cells 
#       - will already have 
#           coordinates of yellow boxes 
#           some of the cells checked 
#   3) go back to starting position

hub = PrimeHub()

color = ColorSensor('E') # change if necessary
distance_front = DistanceSensor('D') # change if necessary
distance_right = DistanceSensor('C') # change if necessary
motor_pair = MotorPair('B', 'A') #### change if necessary


y = 4
x = 6
start = (0,0)
position = (0,0)

red_count = 0
yellow_box_count = 0

red_position = []
yellow_box_position = []
grid = [[None for i in range(y)] for j in range(x)]
 # for recording the red cells on our grid: 0 - not checked; 1 - red cell; 2 - checked+no red cell;
red_cell_matrix = [[0 for i in range(4)] for j in range(6)] 




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
            # Orientation (???)
            break


'''class Orientation(): # maybe not needed

    def __init__(self, orientation = 1):
        self.directions = [(1, 0), (0, 1), (-1, 0), (0, -1)] # N=0, E=1, S=2, W=3
        self.current = orientation
        self.current_direction = self.directions[self.current]
    
    def turn_right(self):
        self.current = (self.current + 1) % 4
        self.current_direction = self.directions[self.current]
    
    def turn_left(self):
        self.current = (self.current - 1) % 4
        self.current_direction = self.directions[self.current]'''


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
        global red_count, red_position, red_cell_matrix

        cell_color = color.get_color()

        if (cell_color == 'red'):
            red_count  += 1
            red_position.append((position.x, position.y))
            red_cell_matrix[position.x][position.y] = 1
        else:
            red_cell_matrix[position.x][position.y] = 2

def check_horizontal(distance_sensor):
        global yellow_box_count

        distance_to_edge = abs (5 - position.x)

        if distance_sensor.get_distance() / 23 < distance_to_edge:
                box_position = min(int(distance_sensor.get_distance() / 23) + 1, 5)
                yellow_box_position.append((box_position, position.y))
                yellow_box_count += 1

def check_vertical(distance_sensor):
        global yellow_box_count
        
        distance_to_edge = 3 - position.x

        if distance_sensor.get_distance() / 23 < distance_to_edge:
                box_position = min(int(distance_sensor.get_distance() / 23) + 1, 3)
                yellow_box_position.append((box_position, position.y))
                yellow_box_count += 1


                    # P A R T   1 ~~~~~~


def search_yellow_boxes(distance_front, distance_right, move):
        global position
        global yellow_box_position, yellow_box_count
        global red_count, red_position, red_cell_matrix         #because we might find red cells while searching for yellow boxes

        # at initial position ~> 
        check_vertical(distance_front)
        check_horizontal(distance_right)

        if yellow_box_count == 2: return yellow_box_position    # case where we find 2 boxes on vert1 and hor1 lines
        elif yellow_box_count == 1:                 # case where we have one box in vert1 line
            if yellow_box_position[0].x == 0:
                  #get to position (1,0)
                  move.turn_right()
                  move.move_straight()
                  move.turn_left()

                  check_vertical(distance_front)

                  if yellow_box_count == 2: return yellow_box_position # if case we find second box on vert2
                  else: 
                        while (distance_front.get_distance() / 23 - position.y > 1):
                            move.move_straight()
                            check_horizontal(distance_right)
                        if yellow_box_count == 2: return yellow_box_position
                        else: 
                            move.turn_right()
                            check_horizontal(distance_front)
                            if yellow_box_count == 2: return yellow_box_position
                            elif yellow_box_count == 1: return yellow_box_position.append((0,2))

        elif yellow_box_count == 0 or (yellow_box_count==1 and yellow_box_position[0].y == 0):
                while (distance_front.get_distance() / 23 - position.y > 1):
                            move.move_straight()
                            check_horizontal(distance_right)
                

                if yellow_box_count == 2: return yellow_box_position
                elif yellow_box_count == 1: 
                    if yellow_box_position[0].y !=3:        # case when the top horizontal line is empty
                        while (distance_front.get_distance() / 23 - position.y > 1):
                            move.move_straight()
                            check_horizontal(distance_right)
                            return yellow_box_position
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

                        return yellow_box_position


                
    

                    # P A R T   2 ~~~~~~


def search_red_cells(distance_front, distance_right, move):
        global red_count, red_position, red_cell_matrix, yellow_box_count, yellow_box_position, position

        for i in range(x):
             for j in range (y):
                  if red_cell_matrix[i][j] == 0:
                       move.move_to_position((i, j))  ##should implement this somehow + it should contain check_color funciton + while going around the yellow boxes
                       
        return red_position
        

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



if __name__ =='__main__':
    initial_position()
    move = Move(('B', 'A'))

    
    print(grid)
    