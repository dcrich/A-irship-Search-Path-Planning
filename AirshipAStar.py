
"""
A* search algorithm. Based off algorithm developed by Baijayanta Roy, 2019
https://github.com/BaijayantaRoy/Medium-Article/blob/master/A_Star.ipynb
Changes made to fit my data and style guide, to be easy to port to JMP's JSL.

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
Find rpelacement for *continue* in loops
"""

"""
Fixes:
Change path function to provide path in better format
figure out how to make grid basically a cylinder, that the first column is repeated at the end and sett equal to the first?
"""



import numpy as np
import math as m



class Coord:
    def __init__(self, parent=None, position=None, coordinates=None):
        self.parent = parent
        self.position = position # position in graph
        """may not need this, may just do calculation from index to coordinate so there's less to store"""
        self.Coordinates = coordinates # real world position, for calculating greatt circle disttance in heuristic
        
        self.g = 0 # distance from start coord to current coord
        self.h = 0
        self.f = 0
    def __eq__(self,other):
        return self.position == other.position
    def calc_heuristic(self,elevationChange,endCoord):
        gcDistanceToEnd = indeces_to_LatLon_to_GreatCircleDistance(self.Coordinates,endCoord)



def path_function(currentNode,maze):
    """Returns found path from start to end"""
    path = []
    noRows, noColumns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(noColumns)] for j in range(noRows)]
    current = currentNode
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1
    return result


##### what is cost?
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
    startNode.g = 0
    startNode.h = 0
    startNode.f = 0
    endNode = Coord(None, tuple(end))
    endNode.g = 0
    endNode.h = 0
    endNode.f = 0

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
    outerIterations = 0
    maxIterations = (len(searchArea) // 2) ** 10

    # what squares do we search . serarch movement is left-right-top-bottom 
    #(4 movements) from every positon

    moveDirections  =  [[-1, 0 ], # go up
                        [ 0, -1], # go left
                        [ 1, 0 ], # go down
                        [ 0, 1 ]] # go right

    numRowsSearchArea, numColSearchArea = np.shape(searchArea)
    
    # Loop until you find the end
    
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
        """update current cruisee altitude"""  

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
            """Refactor this logic to be more porting friendly"""
            if len([visited_child for visited_child in visitedList if visited_child == child]) > 0:
                continue
            
            """change to calculate great circle distance"""
            cost = 1 
            # Create the f, g, and h values
            child.g = currentCoord.g + cost
            ## Heuristic costs calculated here, this is using eucledian distance
            """
            Change heuristic to fit data & problem
            Change from euclidean to great circle, add climb in altitude to that node
            """
            ####################################################################################
            ################ LEFT OFF HERE, MAKING HEURISTIC ###################################
            ####################################################################################
            if searchArea[child.position[0]][child.position[1]]:
            elif :
            else:
            child.calc_heuristic(elevationChange,end)

            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            if len([i for i in visitList if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            visitList.append(child)

def latlon_to_indeces(coord):
    """ 
    Takes coord in LAT,LON 
    Returns coord in needed LON,LAT Indeces
    """
    coordIndeces = [181 + coord[1]], 85 - coord[0]]] 
    return coordIndeces

def indeces_to_LatLon_to_GreatCircleDistance(coordIndeces1,coordIndeces2):
    """ 
    Takes coord in the index format
    Returns great circle distance in meters
    """
    lon1 = 100 * coordIndeces1[0] - 181, 
    lat1 = 85 - 100 * coordIndeces1[1]
    lon2 = 100 * coordIndeces2[0] - 181, 
    lat2 = 85 - 100 * coordIndeces2[1]
    earth_m = 6371
    gcDistance = earth_m * 2 * m.asin(m.sqrt((m.sin((lat1 - lat2) / 2))**2 + m.cos(lat1) * m.cos(lat2) * (m.sin((lon1 - lon2) / 2))**2))
    return gcDistance

# import map, store map
searchArea = np.loadtxt('elevation3D.txt')

# initialize start, end, cruise altitude, flight ceiling
startLatLon = [0, 0] # starting position
endLatLon = [4,5] # ending position
cruiseAltitude = 2000 
flightCeiling = 3300

path = search(searchArea, start=latlon_to_indeces(startLatLon), end=latlon_to_indeces(endLatLon), cruiseAltitude, flightCeiling)
print(path)