#!/usr/bin/env python3
import rospy
from simulation.msg import Station, StationArray, Position
from optimization.Optimizer import Optimizer
from optimization.SimulatedAnnealingOptimizer import SimulatedAnnealing
from path_planning.BFS import BFS
from path_planning.DFS import DFS
from path_planning.A_star import AStar
from initializer import *


class MainOptimizer(Optimizer):
    def __init__(self,order_optimization_algorithm=order_optimization_technique,path_optimization_technique=path_optimization_technique):
        self.stations = create_stations()
        super().__init__()

        # rospy.init_node('optimization_node',anonymous=True)
        # self._pub = rospy.Publisher('station_orders/', StationArray,queue_size=10)
        self.order_optimizer = self.current_order_optimization_algorithm(self.stations,order_optimization_algorithm)
        self.path_optimizer = path_optimization_technique


    
    def current_order_optimization_algorithm(self,stations,optimization_algorithm):
        if optimization_algorithm:

            if optimization_algorithm == "simulated_annealing":
                return SimulatedAnnealing(stations)

            elif optimization_algorithm == "ant_colony":
                return AntColony(stations)

            elif optimization_algorithm == "genetic_algorithm":
                return GeneticAlgorithm(stations)

            elif optimization_algorithm == "grey_wolf":
                return GreyWolf(stations)

            elif optimization_algorithm == "particle_swarm":
                return ParticleSwarm(stations)

            elif optimization_algorithm == "whale_optimization":
                return WhaleOptimization(stations)
            else:
                raise Exception("Given optimization algorithm is wrong")
        else:
            raise TypeError("You must specifiy the required Optimization Algorithm")
    


    def best_station_orders(self):
        """generate the best optimized order of stations that doesn't break some constraints such as stations priority"""
        optimizer = self.order_optimizer
        best_order = optimizer.optimize()
        return best_order
    

    def generate_shortest_path(self):
        """generate shortes 2d path between stations from start to end"""

        shortest_path = []
        next_start = start_point

        if self.path_optimizer == "BFS":

            for station in self.stations:
                current_goal = (station.station_location.x,station.station_location.y)
                planner = BFS(cell_width, cell_height, map_width, map_height, next_start, current_goal)

                current_path = planner.shortest_path()
                shortest_path+=current_path

        elif self.path_optimizer == "DFS":

            for station in self.stations:
                current_goal = (station.station_location.x,station.station_location.y)
                planner = DFS(cell_width, cell_height, map_width, map_height, next_start, current_goal)

                current_path = planner.shortest_path()
                shortest_path+=current_path     

        elif self.path_optimizer == "AStar":

            for station in self.stations:
                current_goal = (station.station_location.x,station.station_location.y)
                planner = AStar(cell_width, cell_height, map_width, map_height, next_start, current_goal)

                current_path = planner.shortest_path()
                shortest_path+=current_path 
        else:
            raise TypeError("You must specifiy the required Path Optimization Technique")        

        shortest_path.append(end_point)
        return shortest_path     

if __name__ == '__main__':
    a = MainOptimizer()
    # print(a.best_station_orders())
    print(a.generate_shortest_path())



        
    


