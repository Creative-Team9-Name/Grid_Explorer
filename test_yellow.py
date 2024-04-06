
def search_yellow_boxes(distance_front, distance_right, move):
        global position, box_count, red_count

        # at initial position ~> 
        check_vertical(distance_front)
        check_horizontal(distance_right)

        if box_count == 2: return grid    # case where we find 2 boxes on vert1 and hor1 lines
        elif box_count == 1 and any(grid[start[0]][j] == 'B' for j in range(y)):          # case where we have one box in vert1 line
                move.turn_right()
                move.move_straight()
                move.turn_left()

                check_vertical(distance_front)

                if box_count == 2: return grid # if case we find second box on vert2
                else: 
                    while (distance_front.get_distance() / 23 - position[1] > 1):
                        move.move_straight()
                        check_horizontal(distance_right)
                    if box_count == 2: return grid
                    else: 
                        move.turn_left()
                        check_horizontal(distance_front)
                        if box_count == 2: return grid
                        elif box_count == 1: 
                            grid[start[0]][abs (start[1]-2)] = 'B'
                            return grid

        else:
                while (distance_front.get_distance() / 23 - position[1] > 1):
                            move.move_straight()
                            check_horizontal(distance_right)
                

                if box_count == 2: return grid
                elif box_count == 1 and any(grid[j][abs(start[1]-3)] == 'B' for j in range(5)): # case when it is on top horizontal line where one yellow box is hidden behind another
                        move.turn_around()
                        move.move_straight()

                        move.turn_left()

                        while (distance_front.get_distance() / 23 - position[1] > 1):
                             move.move_straight()
                        move.turn_left()
                        check_horizontal(distance_front)
                        if box_count == 2: return grid
                        else: 
                            move.move_straight()
                            move.turn_left()
                            check_horizontal(distance_front)
                            return grid
                else:           # when some other horizontal line and the box hidden behind it, we check vertical lines while moving horizontally
                        while (distance_front.get_distance() / 23 - position[1] > 1):
                            move.turn_right()
                            move.move_straight()
                            check_horizontal(distance_right)
                            return grid