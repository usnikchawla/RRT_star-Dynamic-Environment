import numpy as np
from Obstacle import Obstacle
from Tree import Tree
import utils
import time


#########################################
############### Task Setup ##############
#########################################
start = [-12,-12]
goal = [12.,12.]
epsilon = 0.5 #near goal tolerance
goalLoc = [goal[0],goal[1],epsilon]
chaos = 0.05
obst1 = Obstacle('rect',[-5, 5, 2,3], [0,0], chaos*np.eye(2), 1.5, goalLoc = goalLoc)
obst2 = Obstacle('circle',[3,9,2], [0,0], chaos*np.eye(2), 1.5, goalLoc = goalLoc)
obst3 = Obstacle('rect', [8,-5,1,6], [0,0], chaos*np.eye(2), 1.5, goalLoc = goalLoc)
obst4 = Obstacle('rect', [-3,-3,7,1], [0,0], chaos*np.eye(2), 1.5, goalLoc = goalLoc)
obstacles = [obst1, obst2, obst3, obst4] #list of obstacles

xmin, ymin, xmax, ymax = -15,-15,15,15 #grid world borders
maxNumNodes = np.inf #upper limit on tree size 
eta = 1.0 #max branch length
gamma = 20.0 #param to set for radius of hyperball
resolution = 0.1
goalFound = False

#########################################
########### Begin Iterations ############
#########################################
startTime = time.time()

#1. Initialize Tree and growth
print("Initializing RRT* Tree.....")
tree = Tree(start,goal,obstacles,xmin,ymin,xmax,ymax,maxNumNodes,resolution,eta,gamma,epsilon)


#3. Get Solution Path
solPath,solPathID = tree.initGrowth(exhaust = True,FN = False)

####################
# Plot

utils.generate_plot(tree,solPath)
	

####################

#4. Init movement()-->> update pcurID 
solPath,solPathID,dt = tree.nextSolNode(solPath,solPathID)

#5. Begin replanning loop, while pcur is not goal, do...
while np.linalg.norm(tree.nodes[tree.pcurID,0:2] - tree.goal) > tree.epsilon:
	
	utils.generate_plot(tree,solPath)
	#6. Obstacle Updates
	tree.updateObstacles(dt)

	#7. if solPath breaks, reset tree and replan
	if tree.detectCollision(solPath):
		print("********************************************************")
		print("**** Path Breaks, collision detected, Replanning! ******")
		print("********************************************************")
		tree.reset(inheritCost = True)
		solPath,solPathID = tree.initGrowth(exhaust = False,FN = False)

		if solPath is None:
			print("Algorithm terminated ! \nUnable to connect to Goal even after drawing 100000 new samples this iteration ! \n")
			break

	######## END REPLANNING Block #######
	solPath,solPathID,dt = tree.nextSolNode(solPath,solPathID)

print("Total Run Time: {} secs".format(time.time() - startTime))

if solPath is not None:
	costToGoal, goalID = tree.minGoalID()
	print("Final Total Cost to Goal: {}".format(costToGoal))


