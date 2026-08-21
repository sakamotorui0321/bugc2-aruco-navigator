"""Small dependency-free grid planner used by the BugC2 PC controller."""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Iterable, Sequence

GridPoint = tuple[int, int]


def _inside(blocked: Sequence[Sequence[bool]], point: GridPoint) -> bool:
    x, y = point
    return bool(blocked) and 0 <= y < len(blocked) and 0 <= x < len(blocked[0])


def nearest_free(
    blocked: Sequence[Sequence[bool]], point: GridPoint, max_radius: int = 12
) -> GridPoint | None:
    """Return the nearest unblocked cell around *point*, or None."""
    if not _inside(blocked, point):
        return None
    x0, y0 = point
    if not blocked[y0][x0]:
        return point

    queue: deque[tuple[int, int, int]] = deque([(x0, y0, 0)])
    visited = {(x0, y0)}
    while queue:
        x, y, distance = queue.popleft()
        if distance >= max_radius:
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            candidate = (nx, ny)
            if candidate in visited or not _inside(blocked, candidate):
                continue
            if not blocked[ny][nx]:
                return candidate
            visited.add(candidate)
            queue.append((nx, ny, distance + 1))
    return None


def astar(
    blocked: Sequence[Sequence[bool]], start: GridPoint, goal: GridPoint
) -> list[GridPoint]:
    """Find an eight-connected path. An empty list means no safe path."""
    if not blocked or not blocked[0]:
        return []
    start = nearest_free(blocked, start)  # type: ignore[assignment]
    goal = nearest_free(blocked, goal)  # type: ignore[assignment]
    if start is None or goal is None:
        return []
    if start == goal:
        return [start]

    directions = (
        (-1, -1, math.sqrt(2.0)),
        (0, -1, 1.0),
        (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)),
        (0, 1, 1.0),
        (1, 1, math.sqrt(2.0)),
    )
    open_heap: list[tuple[float, float, GridPoint]] = []
    heapq.heappush(open_heap, (0.0, 0.0, start))
    came_from: dict[GridPoint, GridPoint] = {}
    cost_so_far: dict[GridPoint, float] = {start: 0.0}

    while open_heap:
        _, current_cost, current = heapq.heappop(open_heap)
        if current == goal:
            break
        if current_cost > cost_so_far.get(current, math.inf):
            continue
        cx, cy = current
        for dx, dy, step_cost in directions:
            nx, ny = cx + dx, cy + dy
            neighbor = (nx, ny)
            if not _inside(blocked, neighbor) or blocked[ny][nx]:
                continue
            # Do not cut diagonally through the corner of two occupied cells.
            if dx and dy and (blocked[cy][nx] or blocked[ny][cx]):
                continue
            new_cost = current_cost + step_cost
            if new_cost >= cost_so_far.get(neighbor, math.inf):
                continue
            cost_so_far[neighbor] = new_cost
            came_from[neighbor] = current
            heuristic = math.hypot(goal[0] - nx, goal[1] - ny)
            heapq.heappush(open_heap, (new_cost + heuristic, new_cost, neighbor))

    if goal not in came_from:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def line_is_free(
    blocked: Sequence[Sequence[bool]], start: GridPoint, end: GridPoint
) -> bool:
    """Bresenham visibility test used to remove unnecessary A* corners."""
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        if not _inside(blocked, (x0, y0)) or blocked[y0][x0]:
            return False
        if x0 == x1 and y0 == y1:
            return True
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy


def simplify_path(
    blocked: Sequence[Sequence[bool]], path: Iterable[GridPoint]
) -> list[GridPoint]:
    """Greedily keep only waypoints needed to avoid occupied cells."""
    points = list(path)
    if len(points) <= 2:
        return points
    result = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        furthest = anchor + 1
        for candidate in range(anchor + 2, len(points)):
            if not line_is_free(blocked, points[anchor], points[candidate]):
                break
            furthest = candidate
        result.append(points[furthest])
        anchor = furthest
    return result
