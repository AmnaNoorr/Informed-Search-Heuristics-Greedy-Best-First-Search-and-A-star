import heapq

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

sld_admissible = {
    "Arad": 366, "Zerind": 374, "Oradea": 380, "Sibiu": 253, "Timisoara": 329,
    "Lugoj": 244, "Mehadia": 241, "Dobreta": 242, "Craiova": 160,
    "Rimnicu": 193, "Fagaras": 176, "Pitesti": 100, "Bucharest": 0,
}

# Inadmissible heuristic: artificially inflated by 1.5x
sld_inadmissible = {k: v * 1.5 for k, v in sld_admissible.items()}


def a_star(graph, start, goal, h):
    frontier = [(h[start], 0, start, [start])]
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
    path_a, cost_a = a_star(romania_graph, "Arad", "Bucharest", sld_admissible)
    path_b, cost_b = a_star(romania_graph, "Arad", "Bucharest", sld_inadmissible)

    print("=== TASK 2 RESULTS ===")
    print("A* with Admissible Heuristic   :", path_a, "| Cost:", cost_a)
    print("A* with Inadmissible Heuristic :", path_b, "| Cost:", cost_b)
    print("\n" + "=" * 60)
    print("ANALYSIS & EXPLANATION:")
    print("=" * 60)
    print(
        "1. Optimal Path: The path found by the admissible heuristic "
        "(Arad -> Sibiu -> Rimnicu -> Pitesti -> Bucharest) with cost 418 is optimal.\n"
        "2. Why Admissibility Guarantees Optimality:\n"
        "   - An admissible heuristic obeys h(n) <= h*(n) for all nodes, meaning\n"
        "     it never overestimates the true remaining cost to the goal.\n"
        "   - This ensures that node expansion order in A* (by f(n) = g(n) + h(n))\n"
        "     will never cause an optimal path to be bypassed in favor of a suboptimal goal.\n"
        "3. Why the Inadmissible Heuristic Failed:\n"
        "   - By scaling SLD by 1.5x, h'(Rimnicu) becomes 289.5 (inflated by 96.5),\n"
        "     whereas h'(Fagaras) becomes 264.0 (inflated by 88.0).\n"
        "   - The disproportionate overestimation makes the optimal path through Rimnicu\n"
        "     look unnaturally expensive (higher f-value), leading A* to prematurely\n"
        "     select the Fagaras route and output a suboptimal path with cost 450."
    )