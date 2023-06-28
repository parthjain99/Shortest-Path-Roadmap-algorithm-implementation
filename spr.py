import sys
import numpy as np
from collections import defaultdict
import heapq

'''
Report reflexive vertices
'''
def euclid(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def check_cw(P1, P2, P3):
    val = ((P2[1] - P1[1]) * (P3[0] - P2[0])) - ((P2[0] - P1[0]) * (P3[1] - P2[1]))

    ## clockwise direction
    if val > 0 :
        return 1

    ## counter clockwise direction
    elif val < 0:
        return 2
    ## coolinear direction
    else:
        return 0

def isReflexive(P1, P2, P3):
    return check_cw(P1, P2, P3)

def findReflexiveVertices(polygons):
    vertices=[]
    
    # Your code goes here
    # You should return a list of (x,y) values as lists, i.e.
    # vertices = [[x1,y1],[x2,y2],...]
    vertices = []

    for polygon in polygons:

        if isReflexive(polygon[-1], polygon[0], polygon[1]) == 1:
            vertices.append(polygon[0])

            for i in range(0, len(polygon) - 2):
                if isReflexive(polygon[i], polygon[i + 1], polygon[i + 2]) == 1:
                    vertices.append(polygon[i + 1])

        if isReflexive(polygon[-2], polygon[-1], polygon[0]) == 1:
            vertices.append(polygon[-1])

    
    return vertices

def check_orientation(A, B, C, D):
    return check_cw(A, B, C) == check_cw(A, B, D)

def intersect(A,B,C,D):
    return check_cw(A,C,D) != check_cw(B,C,D) and check_cw(A,B,C) != check_cw(A,B,D)

def collision_check(P1, P2, polygons):
    for polygon in polygons:
        for i in range(0, len(polygon)):
            for j in range(0, len(polygon)):
                if not (P1 == polygon[i]) and not (P1 == polygon[j]) and \
                        not (P2 == polygon[i]) and not (P2 == polygon[j]):
                    if intersect(P1, P2, polygon[i], polygon[j]):
                        return False
    return True

def find_bitangent(P1, P2, polygons):

    flag1 = False
    flag2 = False

    for polygon in polygons:
        if P1 in polygon or P2 in polygon:
            for i in range(len(polygon)):
                if i+1 < len(polygon):
                    n_p = polygon[i+1]
                else:
                    n_p = polygon[0]

                if i > 0:
                    p_p = polygon[i - 1]
                else:
                    p_p = polygon[-1]

                if (P1 == polygon[i] and P2 == n_p) or (P2 == polygon[i] and P1 == n_p):
                    return True

                if P1 == polygon[i]:
                    if check_orientation(P2, P1, p_p, n_p):
                        flag1 = True
                elif P2 == polygon[i]:
                    if check_orientation(P1, P2, p_p, n_p):
                        flag2 = True
    return flag1 and flag2

'''
Compute the roadmap graph
'''
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = defaultdict(list)
    
    # Your code goes here
    # You should check for each pair of vertices whether the
    # edge between them should belong to the shortest path
    # roadmap. 
    #
    # Your vertexMap should look like
    # {1: [5.2,6.7], 2: [9.2,2.3], ... }
    #
    # and your adjacencyListMap should look like
    # {1: [[2, 5.95], [3, 4.72]], 2: [[1, 5.95], [5,3.52]], ... }
    #
    # The vertex labels used here should start from 1
    for i in range(0, len(reflexVertices)):
        vertexMap[i + 1] = reflexVertices[i]

    for i in range(1, len(vertexMap) + 1):
        for j in range(1, len(vertexMap) + 1):

            if i == j:
                continue

            A1 = vertexMap[i]
            B1 = vertexMap[j]

            if find_bitangent(A1, B1, polygons) and collision_check(A1, B1, polygons):
                # good edge
                dist = euclid(A1, B1)
                adjacencyListMap[i].append([j, dist])

    return vertexMap, adjacencyListMap

'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.

    queue = []
    heapq.heappush(queue, (0, start, None))
    visited = set()
    parent_dict = {}
    while queue:
        current_dist, node, parent = heapq.heappop(queue)
        if node in visited:
            continue

        visited.add(node)
        parent_dict[node] = parent
        if node == goal:
            pathLength = current_dist
            break

        for n in adjListMap[node]:
            heapq.heappush(queue, (current_dist + n[1], n[0], node))

    if pathLength:
        node = goal
        while node is not None:
            path.append(node)
            node = parent_dict[node]

        path = path[::-1]
    return path, pathLength

'''
Agument roadmap to include start and goal
'''
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    # Your code goes here. Note that for convenience, we 
    # let start and goal have vertex labels 0 and -1,
    # respectively. Make sure you use these as your labels
    # for the start and goal vertices in the shortest path
    # roadmap. Note that what you do here is similar to
    # when you construct the roadmap.

    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    terminalPoints = [[x1, y1], [x2, y2]]
    updatedALMap = adjListMap.copy()

    for i in range(-1, 1):
        adjacent = []
        for m in range(1, len(vertexMap) + 1):
            if collision_check(terminalPoints[i], vertexMap[m], polygons):
                dist = euclid(terminalPoints[i], vertexMap[m])
                adjacent.append([m, dist])
                updatedALMap[m].append([i, dist])
        updatedALMap[i] = adjacent

    return startLabel, goalLabel, updatedALMap
    
    return startLabel, goalLabel, updatedALMap

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append(list(map(float, xys[p].split(','))))
        polygons.append(polygon)

    # Print out the data
    print ("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print (str(polygons[p]))
    print ("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print ("Reflexive vertices:")
    print (str(reflexVertices))
    print ("")

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print ("Vertex map:")
    print (str(vertexMap))
    print ("")
    print ("Base roadmap:")
    print (str(adjListMap))
    print ("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print ("Updated roadmap:")
    print (str(updatedALMap))
    print ("")

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print ("Final path:")
    print (str(path))
    print ("Final path length:" + str(length))
    

    # Extra visualization elements goes here
