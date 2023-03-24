#!/usr/bin/env python3

if __name__ == '__main__':
    from Planner import Planner
else:
    from .Planner import Planner

from queue import Queue

class BFS(Planner):

    def __init__(self,cell_width,cell_height,map_width,map_height,start,goal):

        super().__init__(cell_width,cell_height,map_width,map_height,start,goal)
        self.visited = []

    def shortest_path(self):
        path = Queue()
        path.put([self.start])
        self.visited.append(self.start) 

        while path.qsize():
            #1- get current point 
            current_path = path.get()
            
            #2- get all possible directions of the last point in the current path
            current_point = current_path[-1]
            possible_directions = self.possible_directions(current_point)

            #3- add possible directions to the list
            for direction in possible_directions:

                p = current_path + [direction]
                self.visited.append(direction) # add direction to visied grid

                if direction == self.goal:
                    return p # shortest path

                path.put(p)
            
            #print(possible_directions)
        
        #print(f"No available path between these points, {self.start}, {self.goal}")
        return []







