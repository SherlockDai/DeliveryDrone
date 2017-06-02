'''
Author: Yue Dai
Email Address: daiyue@usc.edu
'''
import sys
import heapq
from collections import deque
# OPEN THE INPUT FILE AND READ LINES
lines = []
with open(sys.argv[2]) as f:
    lines.extend(f.read().splitlines())

# OPEN OUTPUT FILE
fo= open("output.txt", "wb")

#Define Node Class
class Node:
    parent = None
    fuel = 0
    visited = 0
    def __init__(self, name, neighbors, distances):
        self.name = name
        self.neighbors = neighbors
        self.distances = distances

# Define the function to find the index of certain node in nodes list
def findIndexOfNode(nodes, node_name):
    index = 0
    for node in nodes:
        if node.name == node_name:
            return index
        index = index + 1
    return 'EXCEPTION'

# Define the function to sort the children array based on alphabet
def sortByAlphabet(children):
    less = []
    greater = []
    # euqal is not useful in this case but we still implement it for future use
    equal = []
    if len(children) > 1:
        pivot = children[0]
        for child in children:
            if ord(child[2]) < ord(pivot[2]):
                less.append(child)
            if ord(child[2]) == ord(pivot[2]):
                equal.append(child)
            if ord(child[2]) > ord(pivot[2]):
                greater.append(child)
        return sortByAlphabet(less) + equal + sortByAlphabet(greater)
    else:
        return children

# Define BFS
def BFS(nodes, fuel, source, destination):
    sourceNode = nodes[findIndexOfNode(nodes, source)]
    # set the flag which determine if we have found the destination or not
    flag = 1
    # initiate explored list and frontier queue
    explored = []
    frontier = deque()
    currentNode = sourceNode
    while flag:
        children = []
        explored.append(currentNode.name)
        indexOfNeighbor = 0
        for neighbor in currentNode.neighbors:
            if neighbor not in explored:
                # find the cost of the trip between the current node and the current neighbor
                cost = currentNode.distances[indexOfNeighbor]
                cost = int(cost) + currentNode.fuel
                if cost <= fuel:
                    children.append((cost, currentNode, neighbor))
            indexOfNeighbor += 1
        children = sortByAlphabet(children)
        frontier.extend(children)
        while flag:
            if not frontier:
                return nodes
            currentNode_Info = frontier.popleft()
            currentNode = nodes[findIndexOfNode(nodes, currentNode_Info[2])]
            if currentNode.name not in explored:
                currentNode.visited = 1
                # set the parent and feul attribute
                currentNode.parent = currentNode_Info[1]
                currentNode.fuel = currentNode_Info[0]
                break
        if currentNode.name == destination:
            return nodes

# define function to extract the path from results
def extractPath(nodes, source, destination, fuel):
    path = ''
    destinationNode = nodes[findIndexOfNode(nodes, destination)]
    if destinationNode.visited == 1:
        fuelLeft = fuel - destinationNode.fuel
        currentNode = destinationNode
        while currentNode.parent:
            path = ''.join('-' + currentNode.name + path)
            currentNode = currentNode.parent
        path = source + path
        return {'path': path, 'fuel': fuelLeft}
    else:
        return {'path': 'No Path', 'fuel': 'false'}


# Define DFS
def DFS(nodes, fuel, source, destination):
    sourceNode = nodes[findIndexOfNode(nodes, source)]
    # set the flag which determine if we have found the destination or not
    flag = 1
    # initiate explored list and frontier queue
    explored = []
    frontier = []
    currentNode = sourceNode
    while flag:
        children = []
        explored.append(currentNode.name)
        indexOfNeighbor = 0
        for neighbor in currentNode.neighbors:
            if neighbor not in explored:
                # find the cost of the trip between the current node and the current neighbor
                cost = currentNode.distances[indexOfNeighbor]
                cost = int(cost) + currentNode.fuel
                if cost <= fuel:
                    children.append((cost, currentNode, neighbor))
            indexOfNeighbor += 1
        children = sortByAlphabet(children)
        children.reverse()
        frontier.extend(children)
        while flag:
            if not frontier:
                return nodes
            currentNode_Info = frontier.pop()
            currentNode = nodes[findIndexOfNode(nodes, currentNode_Info[2])]
            if currentNode.name not in explored:
                currentNode.visited = 1
                currentNode.parent = currentNode_Info[1]
                currentNode.fuel = currentNode_Info[0]
                break
        if currentNode.name == destination:
            return nodes

def sortByAlphabet_UCS(children):
    less = []
    greater = []
    # euqal is not useful in this case but we still implement it for future use
    equal = []
    if len(children) > 1:
        pivot = children[0]
        for child in children:
            if ord(child[1].name) < ord(pivot[1].name):
                less.append(child)
            if ord(child[1].name) == ord(pivot[1].name):
                equal.append(child)
            if ord(child[1].name) > ord(pivot[1].name):
                greater.append(child)
        return sortByAlphabet(less) + equal + sortByAlphabet(greater)
    else:
        return children

# Define UCS
def UCS(nodes, fuel, source, destination):
    sourceNode = nodes[findIndexOfNode(nodes, source)]
    # set the flag which determine if we have found the destination or not
    flag = 1
    # initiate explored list and frontier queue
    explored = []
    frontier = []
    currentNode = sourceNode
    while flag:
        explored.append(currentNode.name)
        indexOfNeighbor = 0
        for neighbor in currentNode.neighbors:
            if neighbor not in explored:
                # find the cost of the trip between the current node and the current neighbor
                cost = currentNode.distances[indexOfNeighbor]
                cost = int(cost) + currentNode.fuel
                if cost <= fuel:
                    heapq.heappush(frontier, (cost, currentNode, neighbor))
            indexOfNeighbor += 1
        while flag:
            if not frontier:
                return nodes
            currentNode_Info = heapq.heappop(frontier)
            infoOfNodes = [currentNode_Info]
            while frontier:
                nextNode_Info = heapq.heappop(frontier)
                if nextNode_Info[0] == currentNode_Info[0]:
                    infoOfNodes.append(nextNode_Info)
                    continue
                else:
                    heapq.heappush(frontier, nextNode_Info)
                    break
            if len(infoOfNodes) > 1:
                infoOfNodes = sortByAlphabet_UCS(infoOfNodes)
                currentNode_Info = infoOfNodes[0]
                for infoOfNode in infoOfNodes[1:]:
                    heapq.heappush(frontier, infoOfNode)
            currentNode = nodes[findIndexOfNode(nodes, currentNode_Info[2])]
            if currentNode.name not in explored:
                currentNode.visited = 1
                currentNode.parent = currentNode_Info[1]
                currentNode.fuel = currentNode_Info[0]
                break
        if currentNode.name == destination:
            return nodes

# Collect Information from Input File
nodes = []
# First Line determine what kind of Search Tech we are going to use
searchTech = lines[0]
# Then get how many fuel we have for this trip
fuel = int(lines[1])
# Figure out the where we start
source = lines[2]
# Find out the destiantion
destination = lines[3]
for line in lines[4:]:
    neighbors_list = []
    distances_list = []
    neighbors = line.split()
    # get the name of the node
    node_name = neighbors[0]
    # remove ":"
    node_name = node_name.replace(':', '')
    # record all neighbors
    for neighbor in neighbors[1:]:
        #splite the name of neighbors and the distance between the node and its neighbors
        neighbor_info = neighbor.split('-')
        neighbor_name = neighbor_info[0]
        neighbor_distance = neighbor_info[1]
        # delete comma
        neighbor_distance = neighbor_distance.replace(',','')
        # add neighbor and distance to the lists
        neighbors_list.append(neighbor_name)
        distances_list.append(neighbor_distance)
    nodes.append(Node(node_name,neighbors_list,distances_list))
# start searching
if searchTech == 'BFS':
    nodes = BFS(nodes, fuel, source, destination)
    result = extractPath(nodes, source, destination, fuel)
elif searchTech == 'DFS':
    nodes = DFS(nodes, fuel, source, destination)
    result = extractPath(nodes, source, destination, fuel)
elif searchTech == 'UCS':
    nodes = UCS(nodes, fuel, source, destination)
    result = extractPath(nodes, source, destination, fuel)
if result['fuel'] != 'false':
    output = result['path'] + ' ' + str(result['fuel'])
else:
    output = "No Path"
fo.write(output)
