class Graph:
    def __init__(self, n, edges):
        self.n = n  # Number of nodes
        self.edges = edges  # List of edges, each edge is represented as (u, v, cost)
        self.parent = [i for i in range(n+1)]
        self.rank = [0] * (n+1)

    def find(self, u):
        if self.parent[u] != u:
            self.parent[u] = self.find(self.parent[u])
        return self.parent[u]

    def union(self, u, v):
        root_u = self.find(u)
        root_v = self.find(v)
        if root_u != root_v:
            if self.rank[root_u] > self.rank[root_v]:
                self.parent[root_v] = root_u
            elif self.rank[root_u] < self.rank[root_v]:
                self.parent[root_u] = root_v
            else:
                self.parent[root_v] = root_u
                self.rank[root_u] += 1

    def kruskal_mst(self):
        self.edges.sort(key=lambda x: x[2])  # Sort edges by cost
        mst_cost = 0
        for u, v, cost in self.edges:
            if self.find(u) != self.find(v):
                self.union(u, v)
                mst_cost += cost
        return mst_cost

# Example usage:
n = 5  # Number of nodes
edges = [
    (1, 2, 1),
    (1, 3, 4),
    (2, 3, 2),
    (2, 4, 7),
    (3, 5, 3)
]

graph = Graph(n, edges)
min_cost = graph.kruskal_mst()
print("Minimum total cost to make the graph connected:", min_cost)
