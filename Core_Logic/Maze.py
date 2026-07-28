class MazeGrid:
    def __init__(self, width=16, height=16):
        self.width = width

        self.height = height
        
        # Initialize walls: True = Wall, False = No Wall // is there a wall? the boolean is the answer
        
        # This is a nested dictionary
        # A dictionary is a special data structure in python that assigns a single variable with one or more key:value pairs. This is a hashmap 
        self.walls = {(x, y): {'N': False, 'E': False, 'S': False, 'W': False, 'visited': False} 
                      for x in range(width) for y in range(height)}
    
        self.dist = {(x, y): 255 for x in range(width) for y in range(height)}
         #  ^^^ This line follows the Dijkstra path finding algorithm and sets the distance from the start cell and of each cell of the maze to infinity or 255 in this case.  

        self.goals = [(7, 7), (7, 8), (8, 7), (8, 8)] #length of this list  is 4 
        #track visited cells
        self.visited = {(x,y): False for x in range(width) for y in range(height)}


    def update_wall(self, x, y, direction, exists):
        if self.walls[(x, y)][direction] == exists:
            return # No change if there was already a wall there
        self.walls[(x, y)][direction] = exists  # if there was no wall there make that wall exist in the virtual map.
       
        
        # Mirroring to the adjacent cell
        next_x, next_y = x, y
        opposite = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
        if direction == 'N': next_y += 1
        elif direction == 'S': next_y -= 1
        elif direction == 'E': next_x += 1
        elif direction == 'W': next_x -= 1
        if 0 <= next_x < self.width and 0 <= next_y < self.height:
            self.walls[(next_x, next_y)][opposite[direction]] = exists

    def flood_fill(self, is_speed_run=False):
        """Standard Flood Fill to calculate distances to goal."""
        """Flood-fill Dijkstra: Calculates pathing weights to goals."""
        for cell in self.dist: self.dist[cell] = 255
        #This line follows the Dijkstra path finding algorithm and sets the distance from the start cell of each cell of the maze to infinity or 255 in this case.  
        for g in self.goals: self.dist[g] = 0
        # This line does similarly by setting the distance of the goals to zero.
        changed = True
        while changed:
            changed = False
            for x in range(self.width):
                for y in range(self.height):
                    if (x, y) in self.goals: continue
                    if is_speed_run and not self.visited[(x,y)]:
                        continue
                    min_dist = 255
                    for d, is_wall in self.walls[(x, y)].items(): # 'd' as in direction. Character for each of the 4 cardinal directions
                        if not is_wall:
                            next_x, next_y = x, y
                            if d == 'N': next_y += 1
                            elif d == 'S': next_y -= 1
                            elif d == 'E': next_x += 1
                            elif d == 'W': next_x -= 1
                            
                            if 0 <= next_x < self.width and 0 <= next_y < self.height: # is next x and y in the grid?
                                if is_speed_run and not self.visited[(x,y)]:
                                    continue
                                min_dist = min(min_dist, self.dist[(next_x, next_y)] + 1) # assign lowest value btw 255 and current distance to min dist 
                    
                    if self.dist[(x, y)] != min_dist:
                        self.dist[(x, y)] = min_dist
                        # Show distances in the simulator for debugging
                        changed = True
    
    def calculate_turn_angle(self, current, target): # Returns the relative angle to turn in degrees. Positive = Right turn, Negative = Left turn.
        diff = (target - current + 4) % 4
        if diff ==1 : 
            return 90.0 # Turn Right
        elif diff == 3: 
            return -90.0 # Turn Left
        elif diff ==2: 
            return 180.0 # U-Turn
        else:
            return 0.0

    def update_xy(x, y, heading): # Updates grid coordinates based on the direction we just drove.

        if heading == 0: return x, y + 1  # North
        
        elif heading == 1: return x + 1, y  # East
        
        elif heading == 2: return x, y - 1  # South
        
        elif heading == 3: return x - 1, y  # West
        
        return x, y

    def get_best_direction(self, x, y, current_heading): #Finds the unblocked neighboring cell with the lowest flood-fill distance."""
            directions = ['N', 'E', 'S', 'W']
            best_dir = None
            min_val = self.dist[(x, y)] 
            
            for i in range(4):
                d = directions[i]
                if not self.walls[(x, y)][d]: # If there is no wall
                    nx, ny = x, y
                    if d == 'N': ny += 1
                    elif d == 'S': ny -= 1
                    elif d == 'E': nx += 1
                    elif d == 'W': nx -= 1
                    
                    # If the next cell is valid and has a lower distance to the goal
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.dist[(nx, ny)] < min_val:
                            min_val = self.dist[(nx, ny)]
                            best_dir = i
            return best_dir   
        
    def get_shortest_path(self,start=(0,0)):
        path = [start]
        curr = path[0]
        
        # Directions mapping for coordinate math
        offsets = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
        
        while curr not in self.goals: #self.goals = [(7, 7), (7, 8), (8, 7), (8, 8)] #length of this list  is 4 

            best_neighbor = None
            min_dist = self.dist[curr]
            
            # Look at all 4 directions
            for d, (dx, dy) in offsets.items():
                # As long as there is no wall in that direction, 
                # set the position of the best neighbor equal to the position of the neighbor cell.
                if not self.walls[curr][d]: # if there is no wall at the current position and direction set the best neighbor equal to the current position plus direction offset.
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

        return path 
    
        # return the path once one of the goal cells is reached. 
        # control does not leave the while loop until one of 
        # the goal cells are reached or until an invalid best_neighbor exists. 
        # The function thus creates a path (a line or  collection 
        # of (x,y) points), which is a list of tuples that the mouse must visit.
        # This function needs to be ran after the flood_fill() not only because
        # the floodfill() makes sure we get the right distances of each
        # cell to the goal, and the right mapping of walls inside of
        # the maze but also because the function assumes we know where the walls are
        # especially considering we use optimistic mapping (no walls until proven otherwise by sensors). 
