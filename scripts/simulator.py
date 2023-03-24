#!/usr/bin/env python3
import pygame
from initializer import *
from run import *
from path_planning.BFS import BFS
import time


class Simulator:
    def __init__(self,stations_list=[],load_station=(10,10),obstacles_list=[],path_color=(255,0,0),
                start_point=start_point,end_point=end_point,window_size=window_size,rows=rows,cols=cols,cell_width=cell_width,cell_height=cell_height):

        pygame.init()
        self.window = pygame.display.set_mode((window_size,window_size),pygame.RESIZABLE)
        self.window_size = window_size
        self.rows = rows
        self.cols = cols

        self.start_point = start_point
        self.end_point = end_point
        self.path_color = path_color
        self.obstacles_list = obstacles_list
        self.stations_list = stations_list

        self.cell_width = cell_width
        self.cell_height = cell_height

        self.path_optimizer = path_optimization_technique

        pygame.display.set_caption("first Simulation")
    
    def grid(self):
        x,y = 0,0
        for i in range(self.rows):
            x+= self.cell_width
            y+=self.cell_height
            pygame.draw.line(self.window, (255,255,255), (x,0), (x,self.window_size))
            pygame.draw.line(self.window, (255,255,255), (0,y), (self.window_size,y))
    
    def redraw(self):
        self.window.fill((0,0,0))
        self.grid()
        self.draw_stations()
        self.draw_pathes()
        pygame.display.update()
    

    def draw_stations(self):
        for station in self.stations_list:
            pos = station.station_location
            pri = station.station_priority
            station_color = (255, 0, 0) if pri == 3 else (0, 255, 0) if pri == 2 else (0, 0, 255) if pri == 1 else (245, 129, 5)
            x,y = pos.x,pos.y
            pygame.draw.rect(self.window, station_color, pygame.Rect(x, y, self.cell_width+1 , self.cell_height +1))
            # pygame.draw.rect(self.window, (0,255,0), pygame.Rect(x, y, self.cell_width+1 , self.cell_height +1),2) # green border
    

    def draw_pathes(self,color=(255, 255, 0)):

        next_start = start_point
        if self.path_optimizer == "BFS":
            for station in self.stations_list:

                current_goal = (station.station_location.x,station.station_location.y)
                planner = BFS(cell_width, cell_height, map_width, map_height, next_start, current_goal)

                current_path = planner.shortest_path()[1:-1]
                #print(current_path)
                pri = station.station_priority
                station_color = (255, 255, 0) if pri == 3 else (255, 185, 0) if pri == 2 else (255, 110, 0) if pri == 1 else (255, 255, 0)

                for point in current_path:
                    x,y = point
                    pygame.draw.rect(self.window, station_color, pygame.Rect(x, y, self.cell_width+1 , self.cell_height +1))
                    pygame.time.delay(100)
                    pygame.display.update()

                next_start = current_goal
        
        else:
            print("I didn't implement that optimizer yet")

    
    def simulate(self):
        while True:
            pygame.time.delay(100)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit()
            
            self.redraw()
            

if __name__ == "__main__":
    a = MainOptimizer("simulated_annealing")
    stations = a.best_station_orders()
    sim = Simulator(stations_list=stations)
    sim.simulate()


    