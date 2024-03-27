from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import Enum

### every time the robot moves and red_count < check red cell

# TODO: red cell identificator - calculate path to move around the map (avoid yellow box) 
# TODO: movement to search for yellow box - take in mind all the possible special cases
# TODO: going to starting position
# TODO: interaction with robot usign bluetooth to give stating position

### security check
# TODO: look alway foward while moving to see if we are going against something
# TODO: do not go out of the grid
# TODO: check black lines after movement to see if went too far

hub = PrimeHub()

color = ColorSensor('E')
distance = DistanceSensor('F')

x = 4
y = 6
# graph method
# eliminate from graph the cells that contain box
# use dfs to find the best path back home
# traveling-salesman algorithm to explore the grid until we find red cell
# find yellow boxes forst al all!!

position = (0,0)
grid[x][y]

class Orientation(): # maybe not needed

    def __init__(self):
        self.N = (1, 0)
        self.E = (0, 1)
        self.S = (-1, 0)
        self.W = (0, -1)
        self.directions = [self.N, self.E, self.S, self.W]
        self.current = 1
        self.current_direction = self.directions[self.current]
    
    def turn_right(self):
        self.current = (self.current + 1) % 4
        self.current_direction = self.directions[self.current]
    
    def turn_left(self):
        self.current = (self.current - 1) % 4
        self.current_direction = self.directions[self.current]

class Move():

    def __init__(self, wheels): # front_wheels = (right, left)
        self.direction = Orientation()
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
    
# wait for initial position and orientation

red_count = 0
box_count = 0

# check color function that checks color when it's on a new cell and save value in matrix

# while red_count = 2 and box_count = 2:
'''
    def check_color():
        cell_color = color.get_color()
        grid[position.x][position.y] = cell_color

    def check_box(orientation):
        box_distance = distance.get_distance(short_range= False)
        box_position = int(box_distance / 23) + 1
     

    def turn90_clockwise

    def turn90_anticlockwise

    def go_straight

    def go_back


'''
if __name__ =='__main__':
    
    print(grid)
    print("Hello")