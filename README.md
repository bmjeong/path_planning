Famous Path Planning Algorithm code practice

<details>
<summary>1) RRT</summary>
* rrt.py
RRT : Rapidly-exploring Random Tree
 </details>
```
 - pseudocode
   - Define Obstacle, Map, Goal, Initial Position
   - Node has 2 property( position, parent )
   - Node List = [Init node in initial Position]
   - Finished = False
   - While Not Finished:
     - Random pt gen
     - Find Nearest Node
     - Make New node
     - If Not Collision
       - Append New node to Node List
       - If Goal Region
         - Finished = True
         - Path Generation 
```


<details>
<summary>2) RRT* </summary>
* rrt_star.py 
 RRT* : using A start concept, path is shortened
(\*) means the difference between RRT and RRT*
```
 - pseudocode
   - Define Obstacle, Map, Goal, Initial Position
   - Node has 2 property( position, parent )
   - Node List = [Init node in initial Position]
   - Finished = False
   - While Not Finished:
     - Random pt gen
     - Find Nearest Node
     - Make New node
     - If Not Collision
       - Append New node to Node List
       - (*)Find Neighbor
       - (*)If Neighbor
          - (*)Update Parent of new node through evaluation Neighbor nodes
          - (*)Update Parent of neighbor node from original to new node (If it's shorter) --> Rewire
       - If Goal Region
         - Finished = True
         - Path Generation 
```
</details>

<details>
<summary>3) RRT* SMART </summary>
* rrt_star_smart.py
 RRT* SMART : by deleting vertices, path is shortened
(\*) means the difference between RRT* and RRT* SMART
```
 - pseudocode
   - Define Obstacle, Map, Goal, Initial Position
   - Node has 2 property( position, parent )
   - Node List = [Init node in initial Position]
   - Finished = False
   - While Not Finished:
     - Random pt gen
     - Find Nearest Node
     - Make New node
     - If Not Collision
       - Append New node to Node List
       - Find Neighbor
       - If Neighbor
          - Update Parent of new node through evaluation Neighbor nodes
          - Update Parent of neighbor node from original to new node (If it's shorter) --> Rewire
       - If Goal Region
         - Finished = True
         - Path Generation
   - (*) Path shorten (If node in path can be deleted and new edge has no collision with obstacle, then delete node, and Iterate until no node can be deleted)
```
</details>

<details>
<summary>4) PRM </summary>
* prm.py & Util.py
 PRM (Probabilistic RoadMap)
PRM is kind of a framework to find path to avoid collision with obstacles.
1) Vertex Generation
2) Edge Connection
3) Find Shortest Path
4) If no path, then add vertex

There are so many variations of PRM. 
1) Vertex Generation : It could be totally random or Information based. or Halton sequence could be used.
2) Edge Connection : The easiest way is to connect an edge when the distance between two vertex is shorter than threshold.
                     There are many conditions to connect edges.
3) Find Shortest Path : Dijkstra is the most famous and effective algorithm.
4) Iteration : If initial vertices are sufficient, additional vertices would not be necessary. If not, some amount of vertices should be added and edge for new vertices should be added also.

In this code Vertex is generated at totally random position, Edge is connected by Delaunay Triangulation, The shortest path is found by Dijkstra algorithm. 
```
 - pseudocode
   - Define Obstacle, Map, Goal, Initial Position
   - Finished = False
   - Random vertices gen
   - While Not Finished:
     - Connect Edge for vertices with Delaunay triangulation
     - Find shortest path
     - If no path to goal, then add vertex, delete current edges.
```
</details>

<details>
<summary>5) GVD </summary>
* gvd.py & Util.py
GVD(Generalized Voronoi Diagram) is -- (
```
 - pseudocode
   - Define Obstacle, Map, Goal, Initial Position
   - Finished = False
   - -
   - While Not Finished:
     - -
```
</details>
