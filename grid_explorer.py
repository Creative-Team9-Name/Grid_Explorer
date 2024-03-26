from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import Enum

hub = PrimeHub()

hub.light_matrix.show_image('HAPPY')

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
        self.direction = [self.N, self.E, self.S, self.W]
    


class Move():

    def __init__(self, wheels): # front_wheels = (right, left)
        self.direction = Orientation()
        self.motor = MotorPair(wheels[0], wheels[1])
    
    def move_straight(self):
        self.motor.move(23, unit='cm', steering=0, speed=100)
        position = position + self.direction

    def turn_left(self):
        self.direction = self.direction - 1 
        self.motor.move(0, unit='cm', steering=90, speed=50)
    
    def turn_right(self):
        self.direction = self.direction + 1
        self.motor.move(0, unit='cm', steering=-90, speed=50)
    

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