"""
Assignment starter: A* search on a grid.

Read ../guide.md and ../worked_example.md BEFORE you start coding here.

Your job: fill in every function marked TODO. Do not change function
signatures (the tests in test_astar_grid.py rely on them).

Grid legend:
    'S' = start
    'G' = goal
    '#' = wall (cannot be entered)
    '.' = free cell

Run this file directly to see your solver in action:
    python astar_grid.py
"""
import heapq

# The assignment grid. Do not edit this -- your solver must work on this
# AND on any other valid grid (the test file uses different grids too).
ASSIGNMENT_GRID = [
    "S.......",
    ".#..#.#.",
    ".#....#.",
    ".###.##.",
    "...#....",
    "##.#.##.",
    ".....#..",
    ".##...G.",
]

ROWS = len(ASSIGNMENT_GRID)
COLS = len(ASSIGNMENT_GRID[0])


def find_cell(grid, symbol):
    """Return the (row, col) of `symbol` in `grid`. Already implemented."""
    for r, row in enumerate(grid):
        for c, ch in enumerate(row):
            if ch == symbol:
                return (r, c)
    raise ValueError(f"Symbol {symbol!r} not found in grid")


def is_walkable(grid, r, c):
    """Return True if (r, c) is inside the grid and not a wall.

    Already implemented -- use this inside your neighbours() function.
    """
    rows, cols = len(grid), len(grid[0])
    if not (0 <= r < rows and 0 <= c < cols):
        return False
    return grid[r][c] != "#"


def neighbours(grid, node):
    """TODO: yield the valid 4-directional neighbours of `node` in `grid`.

    `node` is a (row, col) tuple. A neighbour is valid if is_walkable()
    returns True for it. Use up/down/left/right moves only (no diagonals).
    """
    row, col = node

    directions = [
        (-1, 0),  # Up
        (1, 0),   # Down
        (0, -1),  # Left
        (0, 1),   # Right
    ]

    for dr, dc in directions:
        next_row = row + dr
        next_col = col + dc

        if is_walkable(grid, next_row, next_col):
            yield (next_row, next_col)


def heuristic(node, goal):
    """TODO: return the Manhattan distance between `node` and `goal`.

    node and goal are (row, col) tuples.
    Manhattan distance = |row1 - row2| + |col1 - col2|.
    This must be admissible for 4-directional grid movement -- explain in
    your submission notes why Manhattan distance satisfies this.
    """
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def reconstruct_path(came_from, current):
    """Rebuild the path from start to `current` using the came_from map.

    Already implemented.
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(grid, start, goal):
    """TODO: implement the A* algorithm.

    Return a tuple: (path, cost)
      - path: list of (row, col) tuples from start to goal, inclusive.
              Return None if no path exists.
      - cost: total path cost (int). Return float('inf') if no path exists.

    Follow the pseudocode in ../guide.md section 4:
      1. Use a heapq-based priority queue keyed on f(n) = g(n) + h(n).
      2. Track g_score for every discovered node.
      3. Track came_from so you can reconstruct the path.
      4. Track a closed set of fully-expanded nodes.
      5. Stop as soon as you POP the goal node from the open list
         (not merely when you first see it as a neighbour).

    Tie-break tip: pushing tuples like (f, -g, row, col, node) onto the
    heap gives you a deterministic tie-break (prefer larger g) -- see the
    worked example solution for this pattern if you get stuck.
    """

    open_heap = []

    start_g = 0
    start_f = start_g + heuristic(start, goal)

    heapq.heappush(
        open_heap,
        (start_f, -start_g, start[0], start[1], start)
    )

    came_from = {}
    g_score = {start: 0}
    closed = set()

    while open_heap:
        current_f, negative_g, row, col, current = heapq.heappop(open_heap)

        current_g = -negative_g

        if current in closed:
            continue

        if current_g != g_score.get(current, float("inf")):
            continue

        if current == goal:
            path = reconstruct_path(came_from, current)
            return path, g_score[current]

        closed.add(current)

        for next_node in neighbours(grid, current):
            if next_node in closed:
                continue

            tentative_g = g_score[current] + 1

            if tentative_g < g_score.get(next_node, float("inf")):
                came_from[next_node] = current
                g_score[next_node] = tentative_g

                next_f = tentative_g + heuristic(next_node, goal)

                heapq.heappush(
                    open_heap,
                    (
                        next_f,
                        -tentative_g,
                        next_node[0],
                        next_node[1],
                        next_node,
                    )
                )

    return None, float("inf")

if __name__ == "__main__":
    start = find_cell(ASSIGNMENT_GRID, "S")
    goal = find_cell(ASSIGNMENT_GRID, "G")
    print(f"Start: {start}, Goal: {goal}")

    path, cost = astar(ASSIGNMENT_GRID, start, goal)

    if path:
        print(f"Path found (cost={cost}):")
        print(" -> ".join(str(p) for p in path))
    else:
        print("No path exists.")
