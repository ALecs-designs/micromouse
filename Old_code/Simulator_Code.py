# I. Imports
import traceback
import API
import sys
# II. NAVIGATION & GRAPH LOGIC
class MazeGrid:  # This class is a virtual, ideal representation of the real maze.
    def __init__(self, width=16, height=16): # This initializes a "maze" object or instance. The default dimensions are 16x16.       
        """
        # To solve the maze, we represent it in a way that allows us to keep track of the walls and distances of each cell to the goal, and the coordinates of the goal cells.
        # The micromouse maze can be represented as a 16x16 grid of cells. With 256 cells in total. Each cell can have walls in the north, east, south, and west directions. 
        # Each cell also has a distance value that represents how far it is from the goal cells. 
        # and each cell's location can be represented by its coordinates (x,y) where x and y are integers from 0 to 15. 

        # The goal cells are typically (check rules relevant to Region 1 of the IEEE) the four center cells of the maze.\
        # In a 16x16 grid maze these cells would be at the following coordinates:  (7,7), (7,8), (8,7), and (8,8).  

        # We found the goal cells using the following mathematical reasoning (from [insert relevant field of STEM] ), that the center cells were located at 
        # the average position or  coordinates of the maze. To follow that reasoning we used the four corners of the maze.
        #  
        # The coordinates of the four corners of the maze are (0,0), (0,15), (15,0), and (15,15).
        # The average of the x coordinates is (0+0+15+15)/4 = 7.5 and the average of the y coordinates is 
        # (0+15+0+15)/4 = 7.5.

        # Since we are working with integer coordinates to map the maze, continous values of 7.5 are invalid (discrete math) , so we use the floor and ceiling function 
        # to round of these averages and get valid integer numbers (or discrete values).
        # Then we use cartesian product (listing every permutation) to list the x-y coordinates of the four center cells as the goal cells (7,7), (7,8), (8,7), (8,8).
        # These coordinates are at the average position of the cells of the maze.

        # Note: 
        # We used an idealized representation of the maze. 
        
        """
        self.width = width
        self.height = height
        # Nested dictionary or hashmap 
        self.walls = {(x, y): {'N': False, 'E': False, 'S': False, 'W': False} #This is the hashmap storing data on the walls of the maze
                      for x in range(width) for y in range(height)}
        self.dist = {(x, y): 255 for x in range(width) for y in range(height)} #This is the hashmap storing data on the distance of each cell to the center of the maze(goal).
        # This line follows sets the distance of each cell from the goal cells to infinity or the maximum allowable value (255 in this case).  
        self.goals = [(7, 7), (7, 8), (8, 7), (8, 8)] #This is the hashmap storing data on the goal of the maze
        # The four center cells of the maze
        #track visited cells
        self.visited = {(x,y): False for x in range(width) for y in range(height)}
    def update_wall(self, x, y, direction, exists):  # This function updates the virtual maze by adding a wall as it is detected by the mouse. The "exists" is of string type but contains "true" or "false" in it. In python any non empty string is considered boolean true. 
        if self.walls[(x, y)][direction] == exists: # No change because the wall is up-to-date, in other words it does not need an update from this function 
            return 
        self.walls[(x, y)][direction] = exists
        if exists:
            API.setWall(x, y, direction.lower()) #These are the red bars in the simulator. 
        # Mirroring to the adjacent cell
        # This is important to make sure that a the presence of a wall is reflected correctly for neighboring cells.
        # Set the coordinates of the next cell 
        next_x, next_y = x, y
        opposite = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
        if direction == 'N': next_y += 1
        elif direction == 'S': next_y -= 1
        elif direction == 'E': next_x += 1
        elif direction == 'W': next_x -= 1
        # if next cell coordinates are valid (i.e. between 0-15)/(inside the mazegrid) update the minimum distance variable
        if 0 <= next_x < self.width and 0 <= next_y < self.height:
            self.walls[(next_x, next_y)][opposite[direction]] = exists



    def flood_fill(self, is_speed_run=False): #This function calculate distances to goal
        for cell in self.dist: self.dist[cell] = 255  # for every cell set the distance to 255
        for g in self.goals: self.dist[g] = 0 # for every goal cell  set the distance to 0
        changed = True 
        while changed:
            changed = False
            #check every cell in the maze grid.
            for x in range(self.width): 
                for y in range(self.height):
                    if (x, y) in self.goals: continue # ignore goal cells.
                    #Skipping unvisited cells on speed run
                    if is_speed_run and not self.visited[(x,y)]:
                        continue
                    min_dist = 255 
                    for d, is_wall in self.walls[(x, y)].items(): # for each direction and wall 
                        if not is_wall: # check there is no wall in that direction, so we can consider moving to the cell that direction.
                            next_x, next_y = x, y 
                            # Set the coordinates of the next cell
                            if d == 'N': next_y += 1  
                            elif d == 'S': next_y -= 1
                            elif d == 'E': next_x += 1
                            elif d == 'W': next_x -= 1
                            # if next cell coordinates are valid (i.e. between 0-15)/(inside the mazegrid) update the minimum distance variable
                            if 0 <= next_x < self.width and 0 <= next_y < self.height:
                                # Prevents flood fill to look at unvisited neighbor cells during speed_run
                                if is_speed_run and not self.visited[(next_x,next_y)]:
                                    continue



                                min_dist = min(min_dist, self.dist[(next_x, next_y)] + 1) # assign lowest value btw 255 and current distance to min dist 
                    # Check if current cell distance is the smallest. If not it update it.            
                    if self.dist[(x, y)] != min_dist:
                        self.dist[(x, y)] = min_dist
                        API.setText(x, y, str(min_dist))
                        changed = True # confirm the change of minimum distance varable to leave.


    def get_shortest_path(self, start=(0,0)):
        path = [start]
        curr = start
        
        # Directions mapping for coordinate math
        offsets = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
        
        while curr not in self.goals:
            best_neighbor = None
            min_dist = self.dist[curr]
            
            # Look at all 4 directions
            for d, (dx, dy) in offsets.items():
                # As long as there is no wall in that direction, 
                # set the neighbor position coordinates 
                # equal to the position coordinates of the neighbor cell.
                if not self.walls[curr][d]: # if there is no wall at the current position and direction set neighbor to current position plus direction offset.
                    neighbor = (curr[0] + dx, curr[1] + dy)
                    
                    # Ensure neighbor coordinate values are valid and 
                    # have a value lower than the current minimal distance so far.
                    if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                        if self.dist[neighbor] < min_dist:
                            # if the distance of the neighbor cell's is less than 
                            # the current minimal distance so far, then set the min_dist equal to it.
                            min_dist = self.dist[neighbor]
                            # set the best neighbor equal to it. 
                            best_neighbor = neighbor
            
            # Safety check: if no neighbor is found, we are trapped
            # because this could mean that their is always a wall 
            # in between the mouse and the adjacent cells or this means 
            # There is an issue with the function. 
            if best_neighbor is None:
                break
            # set the "current position" variable equal to the best neighbor.     
            curr = best_neighbor
            # add that "current position' to the path array. 
            path.append(curr)
        # return the path once one of the goal cells is reached. 
        # control does not leave the while loop until one of 
        # the goal cells are reached or until an invalid best_neighbor exists. 
        # The function thus creates a path (a line in math because it's a collection 
        # of (x,y) points), which is a list of tuples the mouse must visit.
        # this function needs to be ran after the flood_fill() not only because
        # the floodfill() makes sure we get the right distances of each
        # cell to the goal, and the rigth mapping of walls inside 
        # the maze but also because the function assumes we know where the walls
        # especially considering we use optimistic mapping (no walls until proven otherwise by sensors). 
        return path
    
    def smooth(path):
        smooth_path = []
        # this array will hold the instructions

        if len(path) >= 3:
            i = 0       # initialize loop variable
            while i < len(path) - 2:
            #for i in range(len(path)-2):
                curr_x, curr_y  = path[i]
                next_x, next_y= path[i+1]
                target_x, target_y = path[i+2]
                vec_A_dx = next_x - curr_x
                vec_A_dy = next_y - curr_y

                vec_B_dx = target_x - next_x
                vec_B_dy = target_y - next_y
                
                #vec_c_x = target_x - curr_x
                #vec_c_y = target_y - curr_y 

                if (vec_A_dy == 0 and vec_B_dx == 0) or (vec_A_dx == 0 and vec_B_dy == 0):
                    print(f"Corner detected at {path[i+1]}!")
                    i+=2
                    smooth_path.append("diagonal_turn")
                else: 
                    print(f"Straight line through {path[i+1]}!")
                    i+=1
                    smooth_path.append("forward_1")

        return smooth_path
                # if (curr_x-next_x):
                #    moveForwardHalf(1) # 1 cell is 180mm^2, The maze is a 16x16 grid made of 18x18 (cm) squares. 
                #    turnRight45()
                #    moverForwardHalf(1)
                #    turnRight45()   
                #    moveForwardHalf(1) 

# III. SIMULATOR BRIDGE & MOVEMENT
def log(message):#MMS requires logging to stderr.
    print(message, file=sys.stderr)
def main():
    # 3. INITIALIZATION
    maze = MazeGrid()
    x, y = 0, 0
    heading = 0 # Cardinal direction Encoding =>  0:N, 1:E, 2:S, 3:W 
    directions = ['N', 'E', 'S', 'W'] # cardinal directions
    # 4. CONTROL LOOP
    state = "SEARCH_GOAL"
    while True:
        maze.visited[(x,y)] =True
        # A. READ SENSORS
        # Map relative sensor data to absolute maze directions
        if state in ["SEARCH_GOAL", "RETURN_TO_START"]: #  "SPEED_RUN" 
            f_dir = directions[heading]
            r_dir = directions[(heading + 1) % 4]
            l_dir = directions[(heading - 1) % 4]
            # B. UPDATE VIRTUAL MAZE WALLS
            # "API.wall*()"" functions return string values that hold boolean values "True" or "False". These are then are passed to "exists" fields inside other functions.
            maze.update_wall(x, y, f_dir, API.wallFront()) 
            maze.update_wall(x, y, r_dir, API.wallRight())   
            maze.update_wall(x, y, l_dir, API.wallLeft())
            # C. RUN FLOOD FILL ALGORITHM
            maze.flood_fill(is_speed_run= False)
        # D. CHECK IF THE GOAL CELLS ARE REACHED
        if state == "SEARCH_GOAL" and (x, y) in maze.goals:
            log("Goal Reached! Transitioning to Return_to_start...")
            API.setColor(x, y, 'G')
            #break
            maze.goals = [(0,0)]
            maze.flood_fill() # Re calculate all weights bakc to the start
            state = "RETURN_TO_START"
        elif state == "RETURN_TO_START" and (x,y) == (0,0):
            log("Back at Start! Transitioning to Speed_run...")
            API.setColor(x,y,'B')

            #overwrite goal bakc to center for final speed run
            maze.goals = [(7, 7), (7, 8), (8, 7), (8, 8)]
            maze.flood_fill(is_speed_run=True)
            # Generate the raw coordinates
            raw_path = maze.get_shortest_path(start = (0,0))
            #Smooth them into instructions

            speed_run_commands =maze.smooth(raw_path) # error 7/15/2026
            
            state = "SPEED_RUN"
            
        elif state == "SPEED_RUN" and (x,y) in maze.goals:
            log("Speed Run Complete! Maze Solved.")
            API.setColor(x,y,'R')
            break          
        # E. DECIDE NEXT MOVE
        # Find the neighboring cell with the lowest flood-fill value. The flood-fill value is a variable minimum distance.(min_dist)
        best_dir = None # best_dir is really just the best heading. It starts as none.
        min_val = maze.dist[(x, y)] #Before floodfill() is called every cell if given 255 as distance from goal cells. After flood fill, depending on the maze min_val changes. (min_val = minimum_value).
        for i in range(4): # For every cardinal direction block of code updates the x and y coordinates
            d = directions[i]  
            if not maze.walls[(x, y)][d]: # Check for openings (no wall)
                # Set the coordinates of the next cell 
                next_x, next_y = x, y
                if d == 'N': next_y += 1
                elif d == 'S': next_y -= 1
                elif d == 'E': next_x += 1
                elif d == 'W': next_x -= 1
                # if next cell coordinates are valid (i.e. between 0-15)/(inside the mazegrid) update the minimum distance variable and set the new best direction or heading for the next control loop iteration.
                if 0 <= next_x < 16 and 0 <= next_y < 16:
                    if maze.dist[(next_x, next_y)] < min_val:
                        min_val = maze.dist[(next_x, next_y)]
                        best_dir = i
        # 5. EXECUTE MOVEMENT
        if best_dir is not None: # Check if best direction is valid
            while heading != best_dir: #compares best_direction and current heading # Turn Logic
                # Decide shortest turn
                diff = (best_dir - heading + 4) % 4 # Compute the error between current and best heading as an integer.
                if diff == 1: # If the error computes to 1
                    API.turnRight()
                    heading = (heading + 1) % 4
                elif diff == 3:
                    API.turnLeft()
                    heading = (heading - 1) % 4
                else: # 180 degree turn
                    API.turnRight()
                    heading = (heading + 1) % 4
            API.moveForward() # Move Forward
            if heading == 0: y += 1 # Adjust current position
            elif heading == 1: x += 1
            elif heading == 2: y -= 1
            elif heading == 3: x -= 1
        else:
            log(f"Error: No valid path found! Trapped at ({x},{y}). Cell Distance: {maze.dist[(x,y)]} ")
            traceback.print_exc()
            break
if __name__ == "__main__":
    main()