import sys
import re
import math

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None, height=None):
        self.parent = parent
        self.position = position
        self.height = height

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

#function to read ans store text file information
def inputfile():
    f = open(sys.argv[1],'r')
    data = f.readlines()  #creates list with line 1 as a string being data[0] and line 2 being data[1]
    return data

#assigning start and end coordinates
def startxy(data):
    coordinates_string = data[1].rstrip()
    coordinates = re.findall('\d+', coordinates_string ) #converts string to int
    xstart = int(coordinates[0])
    ystart = int(coordinates[1])
    xend = int(coordinates[2])
    yend = int(coordinates[3])
    return xstart, ystart, xend, yend

#create matrix
def matrixcr(size, data):
    matrix = [[ 0 for i in range(size)] for i in range(size)]  # create matrix of 0's
    for k in range(size):
        row_string = data[2+k].rstrip()
        row = re.findall('\d', row_string )
        for j in range(size):
            matrix[0+k][0+j] = int(row[0+j])
    return matrix
    
#A* Search where heuristic is distance from node to end using Pythagorean and g is cost to move from parent to child
def astar(maze, start, end):

    # Create start and end node
    start_node = Node(None, start, maze[start[0]][start[1]] )
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end, maze[start[0]][start[1]])
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list and node expansion count
    open_list = []
    closed_list = []
    nodecost = 0

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        # Find the lowest costing node
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list, and count node expansion
        open_list.pop(current_index)
        closed_list.append(current_node)
        nodecost += 1

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            cost = 0
            while current is not None:
                cost += current.g # cost is only the g of node
                path.append(current.position)
                current = current.parent
            return path[::-1], nodecost, cost # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make height difference between child and parent is less than 4
            if abs(maze[node_position[0]][node_position[1]]-maze[current_node.position[0]][current_node.position[1]]) > 3:
                continue

            # Create new node and append
            new_node = Node(current_node, node_position, maze[node_position[0]][node_position[1]])
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            hdiff = abs(child.height-current_node.height)
            child.g = 1 +hdiff
            child.h = math.sqrt(((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.f > open_node.f:
                    continue

            # Append child to the open list
            open_list.append(child)

        
#best-first search
#same as above A* but when finding the lowest costing node compare heuristic only
def bestsearch(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start, maze[start[0]][start[1]] )
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end, maze[start[0]][start[1]])
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []
    nodecost = 0

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.h < current_node.h:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        nodecost += 1

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            cost = 0
            while current is not None:
                cost += current.g
                path.append(current.position)
                current = current.parent
            return path[::-1], nodecost, cost # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if abs(maze[node_position[0]][node_position[1]]-maze[current_node.position[0]][current_node.position[1]]) > 3:
                continue

            # Create new node
            new_node = Node(current_node, node_position, maze[node_position[0]][node_position[1]])

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            if child in closed_list:
                continue

            # Create the f, g, and h values
            hdiff = abs(child.height-current_node.height)
            child.g = 1 +hdiff
            child.h = math.sqrt(((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.h > open_node.h:
                    continue

            # Add the child to the open list
            open_list.append(child)
            
def main():
    data = inputfile()
    size = int(data[0].rstrip())  #rstrip removes the new line
    xstart, ystart, xend, yend = startxy(data)
    matrix = matrixcr(size, data)
    start = (xstart, ystart)
    end = (xend, yend)
    path, nodecount, cost = astar(matrix, start, end)
    bpath, bnodecount, bcost = bestsearch(matrix, start, end)
    print("For A* Search:")
    print ("Nodes expanded: ", nodecount)
    print("Path:\n", path)
    print ("Cost of path: ", cost)
    print("\nFor Best First Search:")
    print ("Nodes expanded: ", bnodecount)
    print("Path:\n", bpath)
    print ("Cost of path: ", bcost)

main()

