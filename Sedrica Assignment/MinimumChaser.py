from GraphNetwork import Vertex, Graph

# Instantiating the required Graph
graph = Graph()

# This nested loop creates the vertices of the graph
for i in range(1,6):
    for j in range(1,6):
        V = Vertex((i,j))
        V = V.AsTuple()
        graph.AddVertex(V)


# We now define the edges between the vertices one by one
graph.AddEdge((1,1), (2,1), 2, 2)  
graph.AddEdge((2,1), (3,1), 3, 3)
graph.AddEdge((3,1), (4,1), 9, 9)
graph.AddEdge((4,1), (5,1), 4, 4)
graph.AddEdge((1,2), (2,2), 1, 1)
graph.AddEdge((2,2), (3,2), 4, 4)
graph.AddEdge((3,2), (4,2), 3, 3)
graph.AddEdge((4,2), (5,2), 1, 1)
graph.AddEdge((1,3), (2,3), 4, 4)
graph.AddEdge((2,3), (3,3), 6, 6)
graph.AddEdge((3,3), (4,3), 4, 4)
graph.AddEdge((4,3), (5,3), 12, 12)
graph.AddEdge((1,4), (2,4), 12, 12)
graph.AddEdge((2,4), (3,4), 15, 15)
graph.AddEdge((3,4), (4,4), 32, 32)
graph.AddEdge((4,4), (5,4), 40, 40)
graph.AddEdge((1,5), (2,5), 1, 1)
graph.AddEdge((2,5), (3,5), 15, 15)
graph.AddEdge((3,5), (4,5), 9, 9)
graph.AddEdge((4,5), (5,5), 4, 4)
graph.AddEdge((1,1), (1,2), 2, 2)
graph.AddEdge((1,2), (1,3), 3, 3)
graph.AddEdge((1,3), (1,4), 15, 15)
graph.AddEdge((1,4), (1,5), 20, 20)
graph.AddEdge((2,1), (2,2), 1, 1)
graph.AddEdge((2,2), (2,3), 5, 5)
graph.AddEdge((2,3), (2,4), 7, 7)
graph.AddEdge((2,4), (2,5), 3, 3)
graph.AddEdge((3,1), (3,2), 2, 2)
graph.AddEdge((3,2), (3,3), 7, 7)
graph.AddEdge((3,3), (3,4), 12, 12)
graph.AddEdge((3,4), (3,5), 10, 10)
graph.AddEdge((4,1), (4,2), 1, 1)
graph.AddEdge((4,2), (4,3), 11, 11)
graph.AddEdge((4,3), (4,4), 31, 31)
graph.AddEdge((4,4), (4,5), 20, 20)
graph.AddEdge((5,1), (5,2), 5, 5)
graph.AddEdge((5,2), (5,3), 15, 15)
graph.AddEdge((5,3), (5,4), 11, 11)
graph.AddEdge((5,4), (5,5), 5, 5)


# Code to find the minimum distance from (2,2) to (5,5)
start = (2,2)
end = (5,5)
Path = [start]
Cost = 0

while start != end:
    Neighbours = graph.GetNeighbours(start)
    cost_list = []

    for Node in Neighbours:
        cost_list.append(graph.GetCost(start, Node))
    sort_list = sorted(cost_list)
    min_cost = float('inf')
    index = None
    UnvisitedNeighbour = False

    for cost in sort_list:
        neighbour = Neighbours[cost_list.index(cost)]
        if neighbour not in Path:
            index = cost_list.index(cost)
            min_cost = cost
            UnvisitedNeighbour = True
            break
    if not UnvisitedNeighbour:
        break
            
    Cost += min_cost
    Path.append(Neighbours[index])
    start = Neighbours[index]

print(f"Minimum Cost : {Cost}")
print(f"Minimum Path : {Path}")

# This method does not work in giving the path from (2,2) to (5,5) 
# Because it gets to a point where there are no unexplored nodes and the path gets stuck

