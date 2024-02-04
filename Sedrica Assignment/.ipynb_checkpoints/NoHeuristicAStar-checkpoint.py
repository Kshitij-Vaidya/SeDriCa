import heapq

# This is an implementation of the A* Algorithm without including any Heuristic function
class A_Star:
    def __init__(self):
        self.graph = {}

    def AddEdge(self, start, end, cost):
        if start not in self.graph:
            self.graph[start] = []
        if end not in self.graph:
            self.graph[end] = []
        self.graph[start].append((end, cost))
        self.graph[end].append((start, cost))

    def Algorithm(self, start, end):
        open_set = []
        visited = set()
        parent = {}
        g_costs = {start: 0}
        # Since we are not implementing ant heuristic cost for now, we get the total cost and actual cost to be equal
        heapq.heappush(open_set, (0, 0, start))

        while open_set:
            _, g_cost, current = heapq.heappop(open_set)

            if current == end:
                return self.ReconstructPath(start, end, parent, g_costs[current])

            if current in visited:
                continue

            visited.add(current)
            if current in self.graph:
                for neighbour, edge_cost in self.graph[current]:
                    if neighbour not in visited:
                        new_g_cost = g_cost + edge_cost

                        if new_g_cost < g_costs.get(neighbour, float('inf')):
                            g_costs[neighbour] = new_g_cost
                            parent[neighbour] = current
                        heapq.heappush(open_set, (new_g_cost, new_g_cost, neighbour))
                        
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
    graph = A_Star()

    # We now define the edges between the vertices one by one
    graph.AddEdge((1,1), (2,1), 2)  
    graph.AddEdge((2,1), (3,1), 3)
    graph.AddEdge((3,1), (4,1), 9)
    graph.AddEdge((4,1), (5,1), 4)
    graph.AddEdge((1,2), (2,2), 1)
    graph.AddEdge((2,2), (3,2), 4)
    graph.AddEdge((3,2), (4,2), 3)
    graph.AddEdge((4,2), (5,2), 1)
    graph.AddEdge((1,3), (2,3), 4)
    graph.AddEdge((2,3), (3,3), 6)
    graph.AddEdge((3,3), (4,3), 4)
    graph.AddEdge((4,3), (5,3), 12)
    graph.AddEdge((1,4), (2,4), 12)
    graph.AddEdge((2,4), (3,4), 15)
    graph.AddEdge((3,4), (4,4), 32)
    graph.AddEdge((4,4), (5,4), 40)
    graph.AddEdge((1,5), (2,5), 1)
    graph.AddEdge((2,5), (3,5), 15)
    graph.AddEdge((3,5), (4,5), 9)
    graph.AddEdge((4,5), (5,5), 4)
    graph.AddEdge((1,1), (1,2), 2)
    graph.AddEdge((1,2), (1,3), 3)
    graph.AddEdge((1,3), (1,4), 15)
    graph.AddEdge((1,4), (1,5), 20)
    graph.AddEdge((2,1), (2,2), 1)
    graph.AddEdge((2,2), (2,3), 5)
    graph.AddEdge((2,3), (2,4), 7)
    graph.AddEdge((2,4), (2,5), 3)
    graph.AddEdge((3,1), (3,2), 2)
    graph.AddEdge((3,2), (3,3), 7)
    graph.AddEdge((3,3), (3,4), 12)
    graph.AddEdge((3,4), (3,5), 10)
    graph.AddEdge((4,1), (4,2), 1)
    graph.AddEdge((4,2), (4,3), 11)
    graph.AddEdge((4,3), (4,4), 31)
    graph.AddEdge((4,4), (4,5), 20)
    graph.AddEdge((5,1), (5,2), 5)
    graph.AddEdge((5,2), (5,3), 15)
    graph.AddEdge((5,3), (5,4), 11)
    graph.AddEdge((5,4), (5,5), 5)

    start_node = (2,2)
    end_node = (5,5)
    Path, Cost = graph.Algorithm(start_node, end_node)

    if Path:
        print(Path)
        print(Cost)
    else:
        print("No path found")

