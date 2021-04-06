
"""
A* search algorithm. Based off algorithm developed by Baijayanta Roy, 2019
https://github.com/BaijayantaRoy/Medium-Article/blob/master/A_Star.ipynb
Changes made to fit my problem, data, and style guide, to be easy to port to JMP's JSL.

HOW IT WORKS:
- Takes a matrix of elevation data at 1 degree increments around the globe for the graph.
- Any Coordinate (Node) above the designated flight ceiling is treated as an obstacle.
- Indeces of the elevation matrix are converted to lat lon points to be used in the distance
    from start node and the heuristic calculations. 
- Change in elevation is also used in heuristic if above cruise altitude. Airship climbs additional 1000 feet to maintain safe cushion
- Airship current cruise altitude is reset whenever it climbs or drops, needs to be compared to desired altitude
"""

"""
Notes:
JMP has classes, tuples
Find replacement for the *enumerate* function
Find replacement for *continue* in loops
"""

"""
Fixes:
Change path function to provide path in better format
Figure out how to make grid basically a cylinder, that the first column is repeated at the end and sett equal to the first?
- Add weight so if it just was horizontal, make horizontal movement cost more than the veritical
    OR make it look at the other direction first
"""

####################################################################################
################ LEFT OFF HERE, ###################################
####################################################################################

import numpy as np
import math as m
import matplotlib.pyplot as plt 
import os

FLIGHT_CUSHION = 300

class Coord:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position # position in graph
        self.indeces_to_latlon() # real world position, for calculating greatt circle disttance in heuristic
        self.find_came_from()
        self.g = 0 # distance from start coord to current coord
        self.h = 0
        self.f = 0
    
    def __eq__(self,other):
        return self.position == other.position
    
    def calc_heuristic(self, elevationChange, endCoordinates):
        gcDistanceToEnd = self.indeces_to_LatLon_to_GreatCircleDistance(self.Coordinates,endCoordinates)
        self.h = m.sqrt(gcDistanceToEnd**2 + elevationChange**2)
    
    def g_cost_calc(self, prevCoord):
        new_gCost = self.indeces_to_LatLon_to_GreatCircleDistance(prevCoord.Coordinates, self.Coordinates)
        if self.CameFrom == self.parent.CameFrom:
            new_gCost = 1.5 * new_gCost
        self.g = prevCoord.g + new_gCost
    
    def indeces_to_latlon(self):
        """ 
        Takes coord in index format
        Returns coord in LON,LAT 
        """
        self.Coordinates = [84 - self.position[0], self.position[1] - 180]
    
    def indeces_to_LatLon_to_GreatCircleDistance(self, coord1, coord2):
        """ 
        Takes coord
        Returns great circle distance in meters
        """
        lat1 = (m.pi / 180) *  coord1[0]
        lon1 = (m.pi / 180) *  coord1[1]
        lat2 = (m.pi / 180) *  coord2[0]
        lon2 = (m.pi / 180) *  coord2[1]
        earth_m = 6371
        gcDistance = earth_m * 2 * m.asin(m.sqrt((m.sin((lat1 - lat2) / 2))**2 + m.cos(lat1) * m.cos(lat2) * (m.sin((lon1 - lon2) / 2))**2))
        return gcDistance
    def find_came_from(self):
        if self.parent is None:
            self.CameFrom = 1
        else:
            diffArray = np.array(self.position) - np.array(self.parent.position)
            if np.array_equal(diffArray, np.array([0, 1])):
                self.CameFrom = 1
            elif np.array_equal(diffArray, np.array([0,-1])):
                self.CameFrom = 2  #Going west, Came from the east
            elif np.array_equal(diffArray, np.array([1, 0])):
                self.CameFrom = 3 #Going north, Came from the south
            elif np.array_equal(diffArray, np.array([-1,0])):
                self.CameFrom = 4       

        

def path_function(currentNode,maze):
    """Returns found path from start to end"""
    current = currentNode
    path = []
    while current is not None:
        path.append(current.Coordinates)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    return path


def search(searchArea, start, end, cruiseAltitude, flightCeiling):
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param start:
        :param end:
        :return:
    """

    # Create start and end node with initized values for g, h and f
    startNode = Coord(None, tuple(start))
    endNode = Coord(None, tuple(end))

    #set cruise altitude, make sure start and end are valid coordinates
    currentCruiseAltitude = cruiseAltitude
    elevationAtCoord = searchArea[startNode.position[0]][startNode.position[1]] # get elevation at current coor
    if elevationAtCoord >= currentCruiseAltitude and elevationAtCoord < flightCeiling: #climb needed
        currentCruiseAltitude = elevationAtCoord + FLIGHT_CUSHION 
    elif elevationAtCoord > flightCeiling:
        print('Invalid Start, increase flight ceiling or change coordinates..')
        return 0

    if searchArea[endNode.position[0]][endNode.position[1]]> flightCeiling:
        print('Invalid End, increase flight ceiling or change coordinates.')
        return 0

    """Add check for if start and end coordinates are valid lat,lon points, eg not 370 or 100"""

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration. 
    # From here we will find the lowest cost node to expand next
    visitList = []  
    # in this list we will put all node those already explored so that we don't explore it again
    visitedList = [] 
    
    # Add the start node
    visitList.append(startNode)
    
    # Adding a stop condition. This is to avoid any infinite loop and stop 
    # execution after some reasonable number of steps
    """ May need to change to fit data and problem"""
    outerIterations = 0
    maxIterations = (len(searchArea) // 2) ** 10

    # Which coords do we search next, search movement is left-right-top-bottom 
    moveDirections  =  [[-1, 0 ], # go SOUTH
                        [ 1, 0 ], # go North
                        [ 0, -1], # go EAST
                        [ 0, 1 ]] # go WEST
                        

    numRowsSearchArea, numColSearchArea = np.shape(searchArea)
    
    # Loop until end point is found
    while len(visitList) > 0:
        
        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outerIterations += 1    

        # Get the current node
        currentCoord = visitList[0]
        indexCurrentCoord = 0
        for index, item in enumerate(visitList):
            if item.f < currentCoord.f:
                currentCoord = item
                indexCurrentCoord = index
        
        # update current cruise altitude
        elevationAtCoord = searchArea[currentCoord.position[0]][currentCoord.position[1]] # get elevation at current coord
        if elevationAtCoord >= currentCruiseAltitude: #climb needed
            currentCruiseAltitude = elevationAtCoord + FLIGHT_CUSHION 
            

        # if we hit this point return the path such as it may be no solution or 
        # computation cost is too high
        if outerIterations > maxIterations:
            print ("Too many iterations!")
            return path_function(currentCoord,searchArea)

        # Pop current node out off yet_to_visit list, add to visited list
        visitList.pop(indexCurrentCoord)
        visitedList.append(currentCoord)

        # test if goal is reached or not, if yes then return the path
        if currentCoord == endNode:
            return path_function(currentCoord,searchArea)

        # Generate children from all adjacent squares
        children = []

        for newPosition in moveDirections: 
            # Get node position
            coordPosition = (currentCoord.position[0] + newPosition[0], currentCoord.position[1] + newPosition[1])

            # Make sure within map bounds
            """change to make boundaries non existant, ie add checks that make it more of a sphere"""
            if (coordPosition[0] > (numRowsSearchArea - 1) or 
                coordPosition[0] < 0 or 
                coordPosition[1] > (numColSearchArea -1) or 
                coordPosition[1] < 0):
                continue

            # Make sure below flight ceiling, ie make sure the node isnt an obstacle
            if searchArea[coordPosition[0]][coordPosition[1]] >= flightCeiling:
                continue

            # Create new node
            newCoord = Coord(currentCoord, coordPosition)

            # Append
            children.append(newCoord)

        # Loop through children
        for child in children:
            
            # Child is on the visited list (search entire visited list)
            """Refactor this logic to be more JMP friendly"""
            if len([visited_child for visited_child in visitedList if visited_child == child]) > 0:
                continue
            
            # Create the f, g, and h values
            child.g_cost_calc(currentCoord)
            
            # Heuristic costs calculated here, this is using great circle distance and change in altitude distance
            elevationAtCoord = searchArea[child.position[0]][child.position[1]]
            if elevationAtCoord >= currentCruiseAltitude: #climb needed
                elevationChange = FLIGHT_CUSHION + (elevationAtCoord - currentCruiseAltitude) # may need to change currentCruiseAltitude to the altitude of the parent Coord
            else: # no climb needed, maintain elevation
                elevationChange = 0 
            child.calc_heuristic(elevationChange,endNode.Coordinates)

            child.f = child.g + child.h


            # Child is already in the yet_to_visit list and g cost is already lower
            """Refactor this logic to be more JMP friendly"""
            if len([i for i in visitList if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            visitList.append(child)
        

def latlon_to_indeces(coord):
    """ 
    Takes coord in LAT,LON 
    Returns coord in needed LON,LAT Indeces
    """
    coordIndeces = [ 84 - coord[0], 180 + coord[1]]
    return coordIndeces

# import map, store map
searchArea = np.loadtxt('elevation3D.txt')

# initialize start, end, cruise altitude, flight ceiling
startLatLon = [-14, -77] # starting position
endLatLon = [1,-51] # ending position
startLatLon = [54, -59]
endLatLon = [71,23]
cruiseAltitude = 1000 
flightCeiling = 2000 - FLIGHT_CUSHION # make 300 m less than desired, since airship climbs 300m above whatever the elevation is if greater than cruise altitude

path = search(searchArea, latlon_to_indeces(startLatLon), latlon_to_indeces(endLatLon), cruiseAltitude, flightCeiling)
path = np.array(path)
np.savetxt('path.csv',path)
# plt.plot(path[:,1],path[:,0],'r--')
# plt.savefig('idk.png', dpi=300)
os.system(f'say -v {"Victoria"} {"I am done computing."}')