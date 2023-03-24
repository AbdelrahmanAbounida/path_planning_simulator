#!/usr/bin/env python3
import rospy
import yaml
import string
import random
import math
from simulation.msg import Station, StationArray, Position
            
# in the future we gonna replace this method with a static station locations and names
def create_stations():
        """ create list of stations with random locations """

        # create start_station and end_Station
        starting_station = Station()
        starting_pos = Position()
        starting_pos.x = start_point[0]
        starting_pos.y = start_point[1]
        starting_station.station_location = starting_pos
        starting_station.station_priority = 4
        starting_station.station_name = "starting station"

        end_station = Station()
        end_pos = Position()
        end_pos.x = end_point[0]
        end_pos.y = end_point[1]
        end_station.station_location = end_pos
        end_station.station_priority = 0
        end_station.station_name = "end station"

        locations = [starting_pos,end_pos]

        loc_x = list(range(0,rows*cell_width,cell_width)) # list of random locations in x direction
        loc_y = list(range(0,cols*cell_height,cell_height)) # list of random locations in y direction

        station_array = StationArray()
        station_array.stations.append(starting_station)
        station_array.stations.append(end_station)
        
        for name in station_names:
            station = Station()
            # 1- station name
            station.station_name = name
            # 2- station location
            location_x = random.choice(loc_x) # random.randint(1, map_width-1)
            location_y = random.choice(loc_y) # random.randint(1, map_height-1)
            loc = (location_x,location_y)

            while loc in locations :
                location_x = random.choice(loc_x) # random.randint(1, map_width-1)
                location_y = random.choice(loc_y) # random.randint(1, map_height-1)
                loc = (location_x,location_y)

            locations.append(loc)
            pos = Position()
            pos.x = loc[0]
            pos.y = loc[1]

            station.station_location = pos
            # 3- station priority
            station.station_priority = random.randint(1, 3)

            station_array.stations.append(station)
        
        return station_array.stations


def load_parameter_server(yaml_path="./simulation/scripts/config.yaml"):
    "load parameter server data manually if they are not available"

    with open(yaml_path, "r") as yamlfile:
        data = yaml.load(yamlfile, Loader=yaml.FullLoader)
        map = data['map']
        map_width = int(map['width'])
        map_height = int(map['height'])
        rows = int(map['rows'])
        cols = int(map['cols'])

        num_of_stations = int(data["num_of_stations"])
        station_names = list(string.ascii_uppercase[:num_of_stations])
        #station_names = ['A','B','C','D','E','F','G','H','I','J','K']

        window_size = int(data["window_size"])
        cell_width = int(map_width // rows)
        cell_height = int(map_height // cols)
        
        start_point= data["start_point"].split(',')
        start_point = (int(start_point[0]),int(start_point[1]))
        end_point=(map_width-cell_width,map_height-cell_height)

        order_optimization_technique = data["order_optimization_technique"]
        path_optimization_technique = data["path_optimization_technique"]

    return map_width,map_height,rows,cols,num_of_stations,station_names,window_size, \
            cell_width,cell_height,start_point,end_point,order_optimization_technique,path_optimization_technique


try:
    map = int(rospy.get_param("map"))
    map_width = int(map['width'])
    map_height = int(map['height'])
    rows = int(map['rows'])
    cols = int(map['cols'])

    num_of_stations = int(rospy.get_param("num_of_stations"))
    station_names = list(string.ascii_uppercase[:num_of_stations])
    #station_names = ['A','B','C','D','E','F','G','H','I','J','K']

    window_size = int(rospy.get_param("window_size"))

    cell_width = map_width // rows
    cell_height = map_height // cols

    start_point= tuple(map(int,rospy.get_param("start_point")).split(','))
    end_point=(map_width-cell_width,map_height-cell_height)


    order_optimization_technique = rospy.get_param("order_optimization_technique")
    path_optimization_technique = rospy.get_param("path_optimization_technique")

except:
    map_width,map_height,rows,cols,num_of_stations,station_names,window_size, \
            cell_width,cell_height,start_point,end_point,order_optimization_technique,path_optimization_technique = load_parameter_server()