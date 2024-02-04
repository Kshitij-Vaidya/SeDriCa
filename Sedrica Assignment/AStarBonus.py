from GraphNetwork import Vertex, Graph
import heapq

# Beacuse the grid only has two directions of motion, Manhattan Distance will be the appropriate Heuristic Cost Function
def HeuristicCost(vertex1, vertex2):
    X = abs(vertex1[0] - vertex2[0])
    Y = abs(vertex1[1] - vertex2[1])
    return (X + Y)

class AStar:
    def __init__(self):
        self.open_set = [] # Priority Queue
        self.visited = set()
        self.parent = {}

    def Algorithm(self, graph, start, end):
        g_costs = {start: 0}
        # Since we are not implementing ant heuristic cost for now, we get the total cost and actual cost to be equal
        heapq.heappush(self.open_set, (0, 0, start))

        while self.open_set:
            _, g_cost, current = heapq.heappop(self.open_set)

            if current == end:
                return self.ReconstructPath(start, end, self.parent, g_costs[current])

            if current in self.visited:
                continue

            self.visited.add(current)
            if current in graph.adjacent_list:
                for neighbour in graph.GetNeighbours(current):
                    edge_cost = graph.GetCost(current, neighbour)
                    if neighbour not in self.visited:
                        new_g_cost = g_cost + edge_cost
                        estimate_cost = new_g_cost + HeuristicCost(neighbour, end)

                        if estimate_cost < (g_costs.get(neighbour, float('inf')) + HeuristicCost(current, end)):
                            g_costs[neighbour] = new_g_cost
                            self.parent[neighbour] = current
                        heapq.heappush(self.open_set, (new_g_cost, new_g_cost, neighbour))
                        
        return None

    def ReconstructPath(self, start, end, parent, cost):
        current = end
        path = []
        while current != start:
            path.insert(0, current)
            current = parent[current]
        path.insert(0, start)
        return path, cost
    

if __name__ == "__main__":
    graph = Graph()
    pathfinder = AStar()

    for i in range(1, 6):
        for j in range(1,6):
            V = Vertex((i,j))
            graph.AddVertex(V.AsTuple())

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

    start_node = (2,2)
    end_node = (5,5)
    Path, Cost = pathfinder.Algorithm(graph, start_node, end_node)

    if Path:
        print("Shortest Path and Cost found using AStar Algorithm :")
        print(f"Path : {Path}")
        print(f"Cost : {Cost}")
    else:
        print("No path found")



