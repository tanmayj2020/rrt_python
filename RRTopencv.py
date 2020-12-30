from try1 import RRTGraph , RRTMap
from cv2 import waitKey 


def main():
    dimensions = (512,512) #dimensions of the map
    start = (20,20) #start coordinates for the robot 
    goal = (200 ,200) #goal coordinates for robot 
    map = RRTMap(start , goal , dimensions)
    graph = RRTGraph(start , goal , dimensions)
    #make obstacles randomly
    obstacles = graph.makeobs()
    #draw the map
    map.drawMap(obstacles)
    #iteration couonter for biasing
    i = 1    
    while(not graph.path_to_goal(goal)):
        #biasing the tree
        if i % 10 == 0:
            X,Y, parent = graph.bias(goal)
            map.drawNode("N" , [X[-1] , Y[-1] , parent])
            map.drawEdge((X[-1] , Y[-1]) , (X[parent[-1]] , Y[parent[-1]]))
            map.refreshMap()
        #expanding the tree
        else:
            X,Y,parent = graph.expand()
            map.drawNode("N" , [X[-1] , Y[-1] , parent])
            map.drawEdge((X[-1] , Y[-1]) , (X[parent[-1]] , Y[parent[-1]]))
            map.refreshMap()
        i += 1
        
    #get the path coordinates
    graph.path_to_goal(goal)
    #draw the path
    map.drawPath(graph.getPathCoords())
    waitKey(0)
if __name__ == "__main__":
    main()
