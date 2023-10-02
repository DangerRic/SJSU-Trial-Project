# SJSU-Trial-Project
Intellegent Sytems: Autonomous Rover Simulator

#PROJECT OVERVIEW

This project is the design and implementation of a autnomous rover simulator which is tasked to navigate through a set of obstacles. This project helps showcases the principles of robotics and algorithm thinking while working with Python language and Pygame library.

#DESIGN AND IMPLEMENTATION

The simulator was designed as a grid-based representation using Python's Pygame library. The grid uses a starting position, an end goal position, and randomly positioned obstacles that the rover must navigate. 

The Rover and the Obstacles are designed as Python classes, each with a position represented as attributes and methods to draw themselves on the Pygame window. 

The simulator uses the A* pathfinding algorithm to help the rover find the optimal path from the starting location to the end goal while avoiding obstacles. This algorithm uses heuristics and calculates the shortest path for the rover.

Collision detection was implemented to prevent the rover from moving into an obstacle. The rover's pathfinder reroutes if a previously unseen obstacle is encountered.

The rover's movement was programmed to move at least in four different directions (up, down, left, right), and measures were taken to ensure that the rover stays within the Pygame window.

#PERFORMANCE

The simulator is designed to handle each tasks efficiently due to the use of the A* algorithm. This algorithm ensures the path found is the shortest and most efficient, allowing the rover to get to its destination using the least number of steps possible. 

The performance is measured based on metrics such as the number of steps the rover takes to reach the goal and the number of collisions with obstacles it makes. The efficiency of navigation is then calculated as follows: `(number of steps - number of collisions) / number of steps * 100%`. 

#CHALLENGES FACED

1. Pathfinding Implementation: Implementing the A* algorithm for pathfinding was quite challenging due to its complexity. However, with careful study of the algorithm, it was successfully implemented. As well as choosing which algorithm to implement or to take a machine learning approach.

2. Collision Detection: Detecting when the rover was trying to move into an obstacle was another challenge due to the need to keep track of both the rover and obstacle positions. This was overcome by implementing a method that checks for a collision at every move the rover intends to make.

4. Obstacle Management: Ensuring that the rover and the target position were not completely surrounded by obstacles was a challenge. To handle this, a check is made to validate if it's possible for a path to exist before the simulation starts. If no path can be found, the simulation ends.



#HOW TO USE 

To use this project, run the python file `rover_simulator.py`. The rover is displayed as a blue square and the obstacles as black squares. 
The rover starts from the top left corner and tries to reach the green square, which is the goal position.


#INSTRUCTIONS

1. Make sure that you have Python 3.9 and `pygame` library installed on your machine. 
2. Run the "rover_simulator.py" script.
3. A new window will pop up showing the simulation grid. 
4. The Rover will automatically start moving towards the target avoiding the obstacles.


#CHAT GPT HELP

This project was aided by ChatGPT to answer questions and help structure the algorthm

QUESTIONS ASKED

1. what is the difference between A* algorithm and Dijkstra's algorithm and using a machine learning approach for autonomous rover simulations
	-Dijkstra's Algorithm: It aims to find the shortest path from a source node to all other nodes in the graph. It doesn't have a specific goal node in mind initially.
	-A Algorithm*: It is designed to find the shortest path from a source node to a specific goal node. It incorporates heuristics to guide the search toward the goal efficiently.
	-machine learning techniques like reinforcement learning can be used to train the rover to learn optimal paths based on its experience and the environment's characteristics.

2. which algorithm would be more efficient, A* or Dijkstra's?
	- In graphs where edge costs vary, A* is often more efficient than Dijkstra's algorithm. A* takes into account both the known cost from the start to the current node (g(n)) and the estimated cost from the current node to the goal (h(n)), making it more adaptive to different edge costs.

3. explain to me how to implement A* algorithm
	-Initialize Data Structures: 
		Create a priority queue (or min-heap) called the "open set" to store nodes
		Use dictionary or another data structure to keep track of the "came_from" relationship
		Create dictionaries to store the "g-values" and  "f-values"
	-Define the Heuristic Function:
		Define a heuristic function that estimates the cost from each node to the goal node
	-Main Loop:
		Select the node with the lowest f-value (f = g + h) from the open set
	-Expand Neighbors:
		Generate the neighbors of the current node. Ensure that neighbors are within the bounds of the grid (or graph) and are not obstacles.
		For each neighbor, calculate its tentative g-value.
		Check if the tentative g-value is lower than the current g-value of the neighbor
	-Path Reconstruction:
		 the goal node is reached, you can reconstruct the shortest path
