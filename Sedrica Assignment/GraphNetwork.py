# Pre-defining classes for the Vertex and Edge that will be used later in the graph 
class Vertex:
    def __init__(self, id):
        self.id = id

    def AsTuple(self):
        return (self.id[0], self.id[1])

class Edge:
    def __init__(self, source, destination, cost):
        self.source = source
        self.destination = destination
        self.cost = cost 


# Create the Graph Class that contains all the necessary functionalities
class Graph:
    def __init__(self):
        # Create an empty dictionary that will contain the list of all adjacent nodes for a given node
        self.adjacent_list = {}

    # Functions to add a vertex and edge that build the graph
    def AddVertex(self, vertex):
        # Create an empty list in the dictionary that corresponds to each vertex in the graph
        self.adjacent_list[vertex] = []

    def AddEdge(self, V1, V2, cost12, cost21):
        # This function also allows for bidirectional cost functions
        # i.e Different costs depending upon the direction of "travel"
        Edge1 = Edge(V1, V2, cost12)
        Edge2 = Edge(V2, V1, cost21)
        self.adjacent_list[V1].append(Edge1)
        self.adjacent_list[V2].append(Edge2)

    # Function assumes V1 is the source and V2 is the destination
    def GetCost(self, V1, V2):
        for edge in self.adjacent_list[V1]:
            if edge.destination == V2:
                return edge.cost
        return -1
    
    # Returns all the neighbours of a vertex
    def GetNeighbours(self, vertex):
        return [edge.destination for edge in self.adjacent_list[vertex]]
    


