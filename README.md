# Shortest-Path-Roadmap-algorithm-implementation
To run the Algorithm run the below command
```
python visualize.py env_01.txt <start_index_x_coordinate> <start_index_y_coordinate> <destination_index_x_coordinate> <destination_index_y_coordinate>
```
Demo Run Output ->
<img src  = "https://github.com/parthjain99/Shortest-Path-Roadmap-algorithm-implementation/blob/main/Figure_1.png"></img>

### The shortest path roadmap (SPR) algorithm is a path planning algorithm that computes a graph-based representation of an environment to find the shortest path between two given points. The algorithm consists of several steps:

1. **Parsing the input**: The algorithm takes the input file containing polygonal obstacles and parses the obstacle vertices.

2. **Finding reflexive vertices**: Reflexive vertices are the vertices of the polygons that have an inward-facing angle. These vertices are important for constructing the roadmap. The algorithm identifies the reflexive vertices by checking the clockwise orientation of three consecutive vertices.

3. **Constructing the roadmap**: The algorithm creates a graph-based representation of the environment, called the roadmap. The vertices of the roadmap correspond to the reflexive vertices found in the previous step. The algorithm determines which reflexive vertices should be connected by edges in the roadmap. An edge is added between two reflexive vertices if there exists a "bitangent" line segment that connects them without intersecting any obstacles. The bitangent line segment is a line segment that lies entirely inside the free space of the environment.

4. **Updating the roadmap**: The algorithm updates the roadmap by adding the start and goal points as new vertices and connecting them to the existing vertices in the roadmap. This step ensures that the shortest path between the start and goal points can be found within the roadmap.

5. **Performing uniform cost search**: The algorithm performs a uniform cost search on the updated roadmap to find the shortest path between the start and goal points. The uniform cost search explores the roadmap graph using a priority queue based on the accumulated cost of each path.

6. **Outputting the result**: The algorithm outputs the final path as a list of vertex labels and the length of the shortest path.

In the implementation provided, the algorithm is divided into separate functions. The `findReflexiveVertices` function identifies the reflexive vertices. The `computeSPRoadmap` function constructs the roadmap graph. The `updateRoadmap` function adds the start and goal points to the roadmap. The `uniformCostSearch` function performs the uniform cost search. Finally, the main part of the code orchestrates these functions, reads the input, calls the necessary functions, and outputs the result.
