
Environment Setup
Obstacle Creation:
A grid of points is created within specified x, y, and z ranges.
The height (z-value) of each grid point is calculated using a function that creates a smooth, wavy surface with a peak at the center.
The heights are adjusted to ensure they stay within the defined z-range.
Visualization:
The obstacles are visualized using a combination of scatter3 and surf functions to create a 3D scatter plot and surface plot respectively.
The environment is colored using a custom colormap.
Start and Goal Positions:
The start and goal positions are defined and visualized using green markers.
Text labels are added near these positions for clarity.
RRT Algorithm Implementation
Parameters
stepSize: The length of each step in the RRT algorithm.
maxIterTimes: The maximum number of iterations the algorithm will run before stopping.
threshold: The distance threshold for considering the goal reached.
p: Probability for choosing between a random sample and the goal position.
Algorithm Steps
Initialization:
The RRT tree is initialized with the start position.
Various other variables are initialized for tracking iterations, volumes, etc.
Main Loop:
The algorithm iterates until either a path is found or the maximum number of iterations is reached.
In each iteration:
A random sample point is generated within the search space, with a bias towards the goal position based on the probability p.
The nearest node in the RRT tree to the sample point is found.
A new point is calculated in the direction of the sample point from the nearest node, at a distance of stepSize.
A collision check is performed to ensure the new point does not intersect with the obstacles.
If the new point is feasible (i.e., no collision), it is added to the RRT tree.
The algorithm checks if the new point is within the threshold distance to the goal. If yes, the path is found.
Collision Detection:
A function collisionDetec checks if the line segment from the nearest node to the new point intersects with any obstacles by sampling points along the segment and checking their heights against the obstacle surface.
Path Reconstruction:
If a path is found, it is reconstructed by tracing back from the goal node to the start node using the parent indices stored in the RRT tree.
Visualization:
The RRT tree and the final path are visualized in the 3D plot.
Additional Features
Volume Calculation:
An attempt is made to calculate a cumulative "volume" as the RRT expands, influencing the probability p dynamically. This feature is experimental and may need refinement.
Dynamic Probability Adjustment:
The probability p for choosing the goal as a sample point is adjusted dynamically based on the progress of the algorithm.
Usage
Ensure MATLAB is installed on your system.
Copy the provided script into a new MATLAB script file (e.g., RRT_PathPlanning.m).
Run the script in MATLAB.
Observe the 3D plot that shows the obstacles, start and goal positions, RRT tree expansion, and the final path (if found).
Notes
The script uses a simplified collision detection method and volume calculation, which may need to be adjusted or replaced with more accurate methods depending on the specific application.
The algorithm parameters (e.g., stepSize, maxIterTimes, threshold) can be tuned for better performance in different environments.
The script includes basic visualization and debugging output (e.g., iteration count, new point coordinates). This can be expanded or modified for more detailed analysis or customization.
