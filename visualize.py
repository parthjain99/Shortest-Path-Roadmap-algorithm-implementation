import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import spr
'''
Set up matplotlib to create a plot with an empty square
'''

def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        10,          # width
        10,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch
    
'''
Make a patch for the robot
'''
def createPolygonPatchForRobot(polygon):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='gray', lw=1)

    return patch
    

'''
Render polygon obstacles  
'''
def drawPolygons(polygons):
    fig, ax = setupPlot()
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p])
        ax.add_patch(patch)

if __name__ == "__main__":

    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Please provide input file: python visualize.py [env-file] start_x start_y goal_x goal_y")
        exit()
    
    filename = sys.argv[1]

    start_x = int(sys.argv[2])
    start_y = int(sys.argv[3])

    goal_x = int(sys.argv[4])
    goal_y = int(sys.argv[5])

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
    for p in range(0, len(polygons)):
        print(str(polygons[p]))

    # Draw the polygons
    drawPolygons(polygons)

    reflexVertices = spr.findReflexiveVertices(polygons)
    vertexMap, adjListMap = spr.computeSPRoadmap(polygons, reflexVertices)
    start, goal, updatedALMap = spr.updateRoadmap(polygons, vertexMap, adjListMap, start_x, start_y, goal_x, goal_y)
    path, length = spr.uniformCostSearch(updatedALMap, start, goal)

    # for convenience, added start and end to vertexMap
    vertexMap[-1] = [goal_x, goal_y]
    vertexMap[0] = [start_x, start_y]

    # Draw SP Road map in Green
    for key in updatedALMap.keys():
        for val in updatedALMap[key]:
            p1 = vertexMap[key]
            p2 = vertexMap[val[0]]
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='g', linestyle='-', linewidth=3)
            #drawLine(p1[0], p1[1], p2[0], p2[1], "g")

    # Draw Shortest Path in Red
    for i in range(1, len(path)):
        p1 = vertexMap[path[i - 1]]
        p2 = vertexMap[path[i]]
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='r', linestyle='-', linewidth=3)
        ##drawLine(p1[0], p1[1], p2[0], p2[1], "r")

    # ======= delete the above line before you make some changes =======
    plt.show()
    
