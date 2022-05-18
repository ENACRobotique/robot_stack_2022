from robot_nav_enac.Obstacle import Obstacle, Rectangle, Rond
from robot_nav_enac.Points import Points
import math
import time

class AStarGraph(object):
    #Define a class board like grid with two barriers
 
    def __init__(self,liste):
        self.barriers = liste
        #print("LIIIIIIIIIIIIIIIIIISSSSSSSSSTE",liste)
        #self.barriers.append(Rectangle(Points(50,20),Points(80,70)))
        #self.barriers.append(Rectangle(Points(100,20),Points(120,70)))
        #self.barriers.append(Rond(Points(100,50),30))
        #self.barriers.append(Rond(Points(150,75),30))
        #self.barriers.append(Rectangle(Points(50,50),Points(80,80)))
        
        #self.barriers.append(liste)
        #self.barriers = liste_barriers
        #self.barriers.append(Rectangle(Points(149.885,1),Points(150.115,30))) #barre 
      

    #def __init__(self):
    #    self.barriers = []
    #    self.barriers.append(Rond(Points(100,50),30))
    #    self.barriers.append(Rond(Points(150,75),30))

    def heuristic(self, start, goal):
        #Use Chebyshev distance heuristic if we can move one square either
        #adjacent or diagonal
        D = 1
        D2 = math.sqrt(2)
        dx = abs(start.x - goal.x)
        dy = abs(start.y - goal.y)
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)*self.move_cost(start,goal)
 
    def get_vertex_neighbours(self, pos):
        n = []
        #Moves allow link a chess king
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
            x2 = pos.x + dx
            y2 = pos.y + dy
            if x2 < 0 or x2 > 300 or y2 < 0 or y2 > 200:
                continue
            n.append(Points(x2, y2))
        return n
 
    def move_cost(self, a, b):
        for barrier in self.barriers:
            if barrier.IsInObstacle(b):
                return 10000 #Extremely high cost to enter barrier squares

        return 1 #Normal movement cost
 
def IsIn(Poin,liste):
    for P in liste:
        if P == Poin:
            return True

    return False        

def AStarSearch(start, end, graph):

    G = {} #Actual movement cost to each position from the start position
    F = {} #Estimated movement cost of start to end going via this position
 
    #Initialize starting values
    G[start] = 0
    F[start] = graph.heuristic(start, end)
 
    closedVertices = set()  #  closedList = File()
    openVertices = set([start])
    cameFrom = {}
 
    while len(openVertices) > 0:
        #Get the vertex in the open list with the lowest F score
        current = None
        currentFscore = None
        for pos in openVertices:
            if current is None or F[pos] < currentFscore:
                currentFscore = F[pos]
                current = pos
            if current.x >= 638 and current.x <= 650:
                print("here")
                pass
        #Check if we have reached the goal
        if current == end:
            #Retrace our route backward
            path = [current]
            while current in cameFrom:
                current = cameFrom[current]
                path.append(current)
            path.reverse()
            return path, F[end] #Done!
 
        #Mark the current vertex as closed
        openVertices.remove(current)
        closedVertices.add(current)
 
        #Update scores for vertices near the current position
        for neighbour in graph.get_vertex_neighbours(current):
            if IsIn(neighbour,closedVertices): ## toto1
                continue #We have already processed this node exhaustively

            ## toto2
            candidateG = G[current] + graph.move_cost(current, neighbour)
            if IsIn(neighbour,openVertices):
                if candidateG >= G[neighbour]:
                    continue #This G score is worse than previously found"""
                
            #Adopt this G score
            cameFrom[neighbour] = current
            G[neighbour] = candidateG
            H = graph.heuristic(neighbour, end)
            F[neighbour] = G[neighbour] + H
            openVertices.add(neighbour) #Discovered a new vertex
 
    raise RuntimeError("A* failed to find a solution")

if __name__ == "__main__":
    tps1 = time.time()
    liste = [Rond(Points(100,50),30)]
    liste.append(Rond(Points(100,50),30))
    liste.append(Rond(Points(150,75),30))
    liste.append(Rectangle(Points(50,50),Points(80,80)))
    
    
    graph = AStarGraph(liste)

    rapport_x = (1288/(3000))*10
    rapport_y = (880/(2000))*10


    result, cost = AStarSearch(Points(250,150), Points(0,0), graph)
    #print ("route", result)
    #print(tps2 - tps1)
    #print ("cost", cost)
    
    #print(graph.barriers)
    print([v.x for v in result], [v.y for v in result])
    #for barrier in graph.barriers:
    #    plt.plot([v.x for v in barrier], [v.y for v in barrier])

    #print(result[0][1])