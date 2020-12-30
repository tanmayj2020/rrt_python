import cv2
import numpy as np
import random 
import math



#class used for creating the map 
class RRTMap:
    def __init__(self,start , goal , MapDimensions):
        #Start map position 
        self.start = start 
        #Goal map position
        self.goal = goal 
        #Map height and Map width 
        self.MapH , self.MapW = MapDimensions[0] ,MapDimensions[1]
        #Map image which is all pixels of white
        self.MapImg = np.ones([self.MapH , self.MapW , 3] , np.uint8)*255 #white background
        #Map window name 
        self.MapWindowName = "RRT path planning"
        #Node radius for the map 
        self.nodeRad = 0
        #node thickness -1 for fill colour
        self.nodeThickness = -1
        #edge thickness -1 for fill colour
        self.edgeThickness = 2
        #Colours
        self.Black = (10,10,10)
        self.Blue = (255,0,0)
        self.Red = (0,0,255)
        self.Green = (0,255,0)
        self.White = (255,255,255)

    #DrawMap is main method responsible for drawing the map
    def drawMap(self , obstacles):
        self.drawNode("S")
        self.drawNode("G")
        self.drawObs(obstacles)
        #To show the image
        cv2.imshow(self.MapWindowName , self.MapImg)
        cv2.waitKey(0)

    #DrawNode is function for dwaring a node in the image 
    def drawNode(self,nodeType,coords=None):
        if nodeType == "G": #goal node
            cv2.circle(self.MapImg  , (self.goal[0] , self.goal[1]) , 5 , self.Green , self.nodeThickness)
        if nodeType == "S": #start node 
            cv2.circle(self.MapImg , (self.start[0] , self.start[1]) , 5 , self.Green,self.nodeThickness)
        if nodeType == "N":  #normal node 
            cv2.circle(self.MapImg  , (coords[0], coords[1]) , self.nodeRad , self.Red,self.nodeThickness)
        if nodeType == "P": #path node ie which is the final path
            cv2.circle(self.MapImg , (coords[0] , coords[1]) , 3 , self.Black , self.nodeThickness)

    #Draw the obstacles in the map with obstacles being an array with elements being upper corner and lower corner of obstacles     
    def drawObs(self , obstacles):
        #To avoid changing the list itself so we copy the list to new list ie clone the list 
        obstaclesList = obstacles.copy()
        while(len(obstaclesList) > 0):
            upper = obstaclesList.pop(0) #upper corner of the obstacle
            lower = obstaclesList.pop(0) #lower corner of the obstacle
            cv2.rectangle(self.MapImg , (upper[0], upper[1]), (lower[0], lower[1]) , self.Black , -1)

    #draw the edge between two nodes given the coordinates of two nodes
    def drawEdge(self , node1 , node2):
        cv2.line(self.MapImg , (node1[0], node1[1]), (node2[0], node2[1]) , self.Red , self.edgeThickness)
    

    #This function shows and closes the map in 1 millisecond (so it will be used in a loop)
    def refreshMap(self):
        cv2.imshow(self.MapWindowName , self.MapImg)
        cv2.waitKey(1)

    #Draw the path given the path node coordinates
    def drawPath(self , path):
        for node in path:
            self.drawNode("P" , coords=node)
            cv2.imshow(self.MapWindowName , self.MapImg)
            cv2.waitKey(1)
        



#class used for creating the rrt tree
class RRTGraph:
    def __init__(self,nstart , ngoal , MapDimensions):
        (x,y) = nstart
        self.start  = nstart
        self.ngoal = ngoal 
        self.goalFlag = False
        #list which is x and y coordinates of the nodes
        self.x = []
        self.y = []
        #parent is a list which tells at what index the parent index of the node 
        self.parent = []
        self.x.append(x)
        self.y.append(y)
        #Because the start node has the parent itself so the start node ie 0th node has parent 0 
        self.parent.append(0)
        #Obstacles list
        self.obstacles=[]
        self.obsDim = 30
        self.obsNum = 80
        self.maph , self.mapw = MapDimensions[0] , MapDimensions[1]
        self.goalstate = None

    #This function makes a random obstacles(rectangle)   
    def makeRandomRect(self):
        #centers of random rectangles
        centerx= int(random.uniform(self.obsDim/2 , self.mapw - self.obsDim/2))
        centery = int(random.uniform(self.obsDim / 2 , self.maph - self.obsDim/2))
        uppercornerx = (centerx - int(self.obsDim/2))
        uppercornery = (centery - int(self.obsDim/2))
        return [uppercornerx, uppercornery]
    #this function makes a list of obstacles in the format required by RRTMap node
    def makeobs(self):
        obs =[]
        for i in range(self.obsNum):
            upper = self.makeRandomRect()
            obs.append(upper)
            obs.append([upper[0] + self.obsDim , upper[1] + self.obsDim])
        self.obstacles = obs.copy()
        return obs 

    # Add a node to the x and y list , where n is the index to add the node to
    def add_node(self,n,x,y):
        self.x.insert(n,x)
        self.y.insert(n,y)
    #Remove a node from x and y list
    def remove_node(self , n):
        self.x.pop(n)
        self.y.pop(n)
    #add edges between parent and child
    def add_edge(self ,child , parent):
        self.parent.insert(parent , child)
    
    #remove the edges between parent and child

    def remove_edge(self , n):
        self.parent.pop(n)


    #total number of nodes in the tree
    def number_of_nodes(self):
        return len(self.x)
    
    #get the distance between the vertices ie the two nodes
    def metric(self , n1 , n2):
        x1 , y1 , x2 , y2  = float(self.x[n1]) , float(self.y[n1]) , float(self.x[n2]) , float(self.y[n2])
        px = (x1 - x2) ** 2 
        py = (y1 - y2) ** 2
        metric = (px + py) ** (0.5)
        return metric
    
    #get the nearest node
    def nearest(self , n):
        dmin = self.metric(0,n)
        nnear = 0
        for i in range(n):
            if self.metric(i,n) < dmin:
                dmin = self.metric(i,n)
                nnear = i
        return nnear

    #sampling a new node in the envronment
    def sample_envir(self):
        #every place in environment has equal probability to be selected so we use uniform distribution to sample the nodes
        x = int(random.uniform(0 , self.mapw))
        y = int(random.uniform(0 , self.maph))
        return x , y


    #To check whether the new sample is located in free space
    def isFree(self):
        n = self.number_of_nodes() - 1  #To get the new sample
        (x , y) = self.x[n] , self.y[n]
        obs = self.obstacles.copy()
        while(len(obs)>0):
            upper = obs.pop(0)
            lower = obs.pop(0)
            if upper[0] -1 < x < lower[0] + 1 and upper[1] -1 < y < lower[1] + 1:
                self.remove_node(n)
                return False
        return True
    #Checks whether the edge between two nodes crosses an osbtacle
    def crossObstacle(self ,x1,x2,y1,y2):
        obs = self.obstacles.copy()
        while(len(obs)>0):
            upper = obs.pop(0)
            lower = obs.pop(0)
            for i in range(0,101):
                u = i/100
                x = x1 * u + x2*(1-u)
                y = y1 * u + y2*(1-u)
                if upper[0] < x < lower[0] and upper[1] < y < lower[1]:
                    return True
        return False
    #method to connect two nodes
    def connect(self, n1 , n2):
        (x1 , y1) = self.x[n1] , self.y[n1]
        (x2,y2) = self.x[n2] , self.y[n2]
        if self.crossObstacle(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1 , n2)
            return True 
    #Step method to check if distance between the new node and nearest node is greator than some dmax then take some other node ont he same line
    #Same line between new node and nearest node
    def step(self,nnear , nrand , dmax=8):
        dmin = self.metric(nnear , nrand)
        if dmin > dmax:
            xnear , ynear = self.x[nnear] , self.y[nnear]
            xrand , yrand = self.x[nrand] , self.y[nrand]
            px , py = xrand - xnear , yrand - ynear
            theta = math.atan2(py , px)
            x , y = int(xnear + dmax * math.cos(theta)) , int(ynear + dmax * math.sin(theta))
            self.remove_node(nrand)
            self.add_node(nrand , x , y)
            if abs(x-self.ngoal[0])<10 and abs(y-self.ngoal[1])<10:
                self.goalstate = nrand
                self.goalFlag=True


    #Method used to get the path to the goal from start
    def path_to_goal(self , ngoal):
        # #Goal flag set to true only when we reach the goal
        # goalflag = False
        # #find the goal state , so after tree is constructed then the node which iis less than 20 pixel to tree is found 
        # #if node is found within 20 pixels then the goal flag is set to true
        # for i in range(self.number_of_nodes()):
        #     (x , y ) = self.x[i] , self.y[i]
        #     if abs(x - ngoal[0]) < 20 and abs(y - ngoal[1]) < 20:
        #         self.goalstate = i
        #         goalflag = True
        #         break
        # if goalflag:
        #     self.path = []
        #     self.path.append(self.goalstate)
        #     newpos = self.parent[self.goalstate]
        #     #looping over until we find the start postion back from goal 
        #     while(newpos != 0):
        #         self.path.append(newpos)
        #         newpos = self.parent[newpos]
        #     self.path.append(0)
        # #Goal flag to be returned to tell whether we foundthe goal or not yet
        # return goalflag
        if self.goalFlag:
            self.path=[]
            self.path.append(self.goalstate)
            newpos=self.parent[self.goalstate]
            while ( newpos !=0):
                self.path.append(newpos)
                newpos=self.parent[newpos]
            self.path.append(0)
        return self.goalFlag


    #Extract the path coordinates from the ids
    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x ,y = self.x[node] , self.y[node]
            pathCoords.append((x,y))
        return pathCoords
    #Bias towards the goal rrt algorthim we dont bias towards the goal fully because by being so greedy 
    #it may happen there is no space to go to goal and
    #we only bias it 5-10% ie ony out of 100 times it will go 90 times branch in all all directions and only 
    #5 tmes it will be biased towards goal
    def bias(self , ngoal):
        #since our indexing begins from 0 this n below will give the index of new node to be added 
        #new node is n which will be added to tree
        n = self.number_of_nodes()
        self.add_node(n , ngoal[0] , ngoal[1])
        nnear = self.nearest(n)
        #So hence we add a new node in direction of goal node making it biased towards goal
        self.step(nnear , n)
        self.connect(nnear , n)
        return self.x , self.y , self.parent
    
    #expansion will sample the environment without biasing
    def expand(self):
        n = self.number_of_nodes()
        x ,y = self.sample_envir()
        self.add_node(n , x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest , n)
            self.connect(xnearest , n)
        return self.x , self.y , self.parent
