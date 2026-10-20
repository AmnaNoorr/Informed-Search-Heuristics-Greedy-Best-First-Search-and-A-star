from collections import deque
import heapq

# Target Goal State
GOAL_STATE = (1, 2, 3, 4, 5, 6, 7, 8, 0)

# Precompute target coordinates for each tile (1-8) for fast Manhattan lookup
GOAL_POSITIONS = {
    1: (0, 0), 2: (0, 1), 3: (0, 2),
    4: (1, 0), 5: (1, 1), 6: (1, 2),
    7: (2, 0), 8: (2, 1), 0: (2, 2)
}


def manhattan_distance(state):
    """Calculates sum of Manhattan distances of tiles 1-8 to their target positions."""
    distance = 0
    for idx, tile in enumerate(state):
        if tile != 0:
            r, c = idx // 3, idx % 3
            target_r, target_c = GOAL_POSITIONS[tile]
            distance += abs(r - target_r) + abs(c - target_c)
    return distance


def get_successors(state):
    """Generates valid neighbor states along with the move direction."""
    zero_idx = state.index(0)
    r, c = zero_idx // 3, zero_idx % 3
    moves = [(-1, 0, "Up"), (1, 0, "Down"), (0, -1, "Left"), (0, 1, "Right")]
    successors = []

    for dr, dc, direction in moves:
        nr, nc = r + dr, c + dc
        if 0 <= nr < 3 and 0 <= nc < 3:
            new_zero_idx = nr * 3 + nc
            new_state = list(state)
            new_state[zero_idx], new_state[new_zero_idx] = new_state[new_zero_idx], new_state[zero_idx]
            successors.append((tuple(new_state), direction))

    return successors


def solve_bfs(start_state):
    """Solves 8-puzzle using Breadth-First Search."""
    queue = deque([(start_state, [])])
    visited = {start_state}
    nodes_expanded = 0

    while queue:
        state, moves = queue.popleft()

        if state == GOAL_STATE:
            return moves, nodes_expanded

        nodes_expanded += 1
        for successor, move in get_successors(state):
            if successor not in visited:
                visited.add(successor)
                queue.append((successor, moves + [move]))

    return None, nodes_expanded


def solve_astar_manhattan(start_state):
    """Solves 8-puzzle using A* Search with Manhattan Distance heuristic."""
    counter = 0  # Tie-breaker for heap
    frontier = [(manhattan_distance(start_state), counter, 0, start_state, [])]
    best_g = {start_state: 0}
    nodes_expanded = 0

    while frontier:
        _, _, g, state, moves = heapq.heappop(frontier)

        if state == GOAL_STATE:
            return moves, g, nodes_expanded

        # Expand node only if current g is valid
        if g > best_g.get(state, float("inf")):
            continue

        nodes_expanded += 1

        for successor, move in get_successors(state):
            new_g = g + 1
            if successor not in best_g or new_g < best_g[successor]:
                best_g[successor] = new_g
                h = manhattan_distance(successor)
                counter += 1
                heapq.heappush(frontier, (new_g + h, counter, new_g, successor, moves + [move]))

    return None, float("inf"), nodes_expanded


if __name__ == "__main__":
    # Test instance from Lab 4 (Standard solvable 8-puzzle instance)
    # Example state: 1 2 3 / 4 0 6 / 7 5 8 (Solvable in 2 moves)
    start_state = (1, 2, 3, 4, 0, 6, 7, 5, 8)

    # Run BFS
    bfs_moves, bfs_expanded = solve_bfs(start_state)

    # Run A*
    astar_moves, astar_cost, astar_expanded = solve_astar_manhattan(start_state)

    print("=== TASK 3 RESULTS (8-PUZZLE) ===")
    print(f"Initial State       : {start_state[:3]}")
    print(f"                      {start_state[3:6]}")
    print(f"                      {start_state[6:]}\n")

    print(f"BFS Solution Length : {len(bfs_moves)} moves")
    print(f"BFS Nodes Expanded  : {bfs_expanded}\n")

    print(f"A* Solution Length  : {len(astar_moves)} moves")
    print(f"A* Sequence of Moves: {astar_moves}")
    print(f"A* Path Cost        : {astar_cost}")
    print(f"A* Nodes Expanded   : {astar_expanded}\n")

    print("-" * 50)
    print("COMPARISON & CONCLUSION:")
    print("-" * 50)
    print(
        f"1. Both BFS and A* find an optimal path of cost {astar_cost}.\n"
        f"2. Search Efficiency: A* expands significantly fewer nodes ({astar_expanded}) "
        f"compared to BFS ({bfs_expanded}) because the Manhattan distance heuristic "
        f"guides the search directly toward the goal instead of exploring blind level-by-level."
    )