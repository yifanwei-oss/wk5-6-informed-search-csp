# NOTES

## Why the A* heuristic is admissible

The heuristic uses Manhattan distance, which calculates the row difference plus the column difference. Since the grid only allows up, down, left, and right movements, the actual path cannot be shorter than the Manhattan distance. Therefore, the heuristic never overestimates the real cost and is admissible.

## Test-case coverage

The A* tests cover an obstacle detour, a boundary case where the start equals the goal, and an unsolvable blocked grid. The CSP tests cover an adjacency conflict, the unconstrained Tasmania region, and an unsolvable problem with only one colour. These tests were selected to cover normal, boundary, and failure behaviours.