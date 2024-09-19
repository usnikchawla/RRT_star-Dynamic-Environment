
# Dynamic Path Planning with RRT*: A Motion Planning Approach

## Overview

This project implements the **Rapidly-exploring Random Tree Star (RRT*)** algorithm for dynamic path planning in evolving environments. RRT* is a powerful and efficient sampling-based algorithm widely used in robotics for solving complex motion planning challenges. In this project, we test RRT* in both static and dynamic environments, where obstacles or goals may change over time.

The algorithm is implemented in Python and visualized using various plotting libraries. The results are visualized as 2D plots showcasing the paths generated and how they adjust in response to environmental changes.

## Theoretical Background

### RRT* Algorithm

The **Rapidly-exploring Random Tree (RRT)** algorithm is designed to efficiently explore high-dimensional, non-convex spaces by incrementally building a space-filling tree. RRT* improves upon RRT by ensuring that the path found converges towards an optimal solution as more samples are added. This makes it highly effective in scenarios involving dynamic environments.

#### Key Concepts:
- **Random Sampling:** The algorithm generates random samples within the configuration space and connects them to the tree in a way that promotes fast exploration.
- **Nearest Neighbor Selection:** The nearest node in the tree is selected and extended towards the random sample, expanding the search space incrementally.
- **Rewiring:** RRT* continuously checks whether adding a new node improves the path cost for other nearby nodes, rewiring the tree for optimality.
- **Asymptotic Optimality:** As the number of samples grows, RRT* ensures that the generated path approaches the optimal solution.

### Dynamic Environment

In this project, the environment is **dynamic**, meaning obstacles and/or goals can change their positions over time. The RRT* algorithm is adapted to respond to these changes by replanning and updating the tree structure, which is crucial in real-world applications like autonomous navigation.

## Prerequisites

The following Python packages are required:

- **NumPy**: For numerical computations.
- **OpenCV (cv2)**: For handling image-based tasks.
- **imageio**: For reading and writing image files.
- **Matplotlib**: For generating plots and visualizing the results.

To install the required dependencies, run the following command:

```bash
pip install numpy opencv-python imageio matplotlib
```

Ensure that all pip packages are up-to-date and activate the Conda environment (if using Conda).

## How to Run the Code

### 1. Set the Working Directory
Ensure that the necessary files are placed in the working directory or update the file paths in the script accordingly.

### 2. Run the RRT* Algorithm

Open a terminal, navigate to the project’s root directory, and execute the following command:

```bash
python3 rrt_star.py
```

This will execute the RRT* algorithm, generating a visual output that shows how the algorithm finds an optimal path in a dynamic environment.

### 3. Output Visualization

The RRT* algorithm will generate 2D plots that visualize the path planning process. If the environment is dynamic, the path will adjust in response to changes such as moving obstacles or goals.

## Folder Structure

```
.
├── rrt_star.py          # Main script implementing RRT* for dynamic environments
├── utils.py             # Utility functions for environment and path handling
├── data/                # Folder for input datasets (if applicable)
└── output/              # Folder for saving output visualizations
```

## RRT* in Dynamic Environments: Theory and Applications

In dynamic environments, the positions of obstacles and goals may change over time, necessitating real-time updates to the planned path. To adapt RRT* to these changes, the following techniques are employed:
- **Replanning:** The algorithm continually checks if the current path remains valid as the environment changes, and if not, it attempts to find a new path.
- **Tree Pruning:** If an obstacle interferes with part of the tree, affected nodes are removed, and the tree is rewired to avoid collisions.

### Real-World Applications:
- **Autonomous Vehicles:** RRT* is widely used for navigation in unpredictable environments where the vehicle must avoid moving obstacles while traveling to its destination.
- **Robotic Manipulation:** Robots often encounter dynamic environments where objects may move, requiring constant path adjustment.

## Example Applications in this Project

- **Task 1:** Static RRT* - The environment remains fixed, and the algorithm finds an optimal path based on static obstacles.
- **Task 2:** Dynamic RRT* - Obstacles and goals may move, and the algorithm dynamically updates the path to ensure optimality.

## License

[Include license information if applicable.]

---

Developed as part of ENPM661 Project 5.
