import heapq

# Graph representation of the Romania road map
romania_graph = {
    "Arad": [("Zerind", 75), ("Sibiu", 140), ("Timisoara", 118)],
    "Zerind": [("Arad", 75), ("Oradea", 71)],
    "Oradea": [("Zerind", 71), ("Sibiu", 151)],
    "Sibiu": [("Arad", 140), ("Oradea", 151), ("Fagaras", 99), ("Rimnicu", 80)],
    "Timisoara": [("Arad", 118), ("Lugoj", 111)],
    "Lugoj": [("Timisoara", 111), ("Mehadia", 70)],
    "Mehadia": [("Lugoj", 70), ("Dobreta", 75)],
    "Dobreta": [("Mehadia", 75), ("Craiova", 120)],
    "Craiova": [("Dobreta", 120), ("Rimnicu", 146), ("Pitesti", 138)],
    "Rimnicu": [("Sibiu", 80), ("Craiova", 146), ("Pitesti", 97)],
    "Fagaras": [("Sibiu", 99), ("Bucharest", 211)],
    "Pitesti": [("Rimnicu", 97), ("Craiova", 138), ("Bucharest", 101)],
    "Bucharest": [("Fagaras", 211), ("Pitesti", 101)],
}

# Straight-Line Distance (SLD) heuristic to Bucharest
sld_admissible = {
    "Arad": 366, "Zerind": 374, "Oradea": 380, "Sibiu": 253, "Timisoara": 329,
    "Lugoj": 244, "Mehadia": 241, "Dobreta": 242, "Craiova": 160,
    "Rimnicu": 193, "Fagaras": 176, "Pitesti": 100, "Bucharest": 0,
}


def compute_path_cost(graph, path):
    """Utility function to calculate total path cost for Greedy Best-First Search."""
    total_cost = 0
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        for neighbor, cost in graph[u]:
            if neighbor == v:
                total_cost += cost
                break
    return total_cost


def greedy_best_first(graph, start, goal, h):
    frontier = [(h[start], start, [start])]
    visited = set()

    while frontier:
        _, node, path = heapq.heappop(frontier)

        if node == goal:
            return path

        if node in visited:
            continue
        visited.add(node)

        for neighbor, _cost in graph.get(node, []):
            if neighbor not in visited:
                heapq.heappush(frontier, (h[neighbor], neighbor, path + [neighbor]))

    return None


def a_star(graph, start, goal, h):
    frontier = [(h[start], 0, start, [start])]  # (f, g, node, path)
    best_g = {start: 0}

    while frontier:
        f, g, node, path = heapq.heappop(frontier)

        if node == goal:
            return path, g

        for neighbor, cost in graph.get(node, []):
            new_g = g + cost
            if neighbor not in best_g or new_g < best_g[neighbor]:
                best_g[neighbor] = new_g
                f_new = new_g + h[neighbor]
                heapq.heappush(frontier, (f_new, new_g, neighbor, path + [neighbor]))

    return None, float("inf")


if __name__ == "__main__":
    start_city = "Arad"
    goal_city = "Bucharest"

    # Greedy Best-First Search
    gbfs_path = greedy_best_first(romania_graph, start_city, goal_city, sld_admissible)
    gbfs_cost = compute_path_cost(romania_graph, gbfs_path)

    # A* Search
    astar_path, astar_cost = a_star(romania_graph, start_city, goal_city, sld_admissible)

    print("=== TASK 1 RESULTS ===")
    print(f"Greedy Best-First Search Path : {' -> '.join(gbfs_path)}")
    print(f"Greedy Best-First Search Cost : {gbfs_cost}\n")
    print(f"A* Search Path                : {' -> '.join(astar_path)}")
    print(f"A* Search Cost                : {astar_cost}")