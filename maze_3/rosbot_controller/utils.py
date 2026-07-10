"""Small math/grid helper functions.

Extracted from the provided working controller without logic changes.
"""

import math

from config import *

# ============================================================
# General helpers
# ============================================================


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
    # this function was written with the help of ChatGPT at our specific request and editorial remarks


def bresenham_line(start_col, start_row, end_col, end_row):
    """Return all (column, row) cells on a grid line."""

    cells = []
    delta_col = abs(end_col - start_col)
    delta_row = abs(end_row - start_row)
    step_col = 1 if start_col < end_col else -1
    step_row = 1 if start_row < end_row else -1
    error = delta_col - delta_row

    current_col = start_col
    current_row = start_row

    while True:
        cells.append((current_col, current_row))

        if current_col == end_col and current_row == end_row:
            break

        double_error = 2 * error

        if double_error > -delta_row:
            error -= delta_row
            current_col += step_col

        if double_error < delta_col:
            error += delta_col
            current_row += step_row

    return cells
    # this function was written with the help of ChatGPT at our specific request and editorial remarks


def euclidean_cells(cell_a, cell_b):
    return math.hypot(cell_a[0] - cell_b[0], cell_a[1] - cell_b[1])
    # this function was written with the help of ChatGPT at our specific request and editorial remarks


def octile_heuristic(cell_a, cell_b):
    """Admissible heuristic for an 8-connected grid."""

    delta_row = abs(cell_a[0] - cell_b[0])
    delta_col = abs(cell_a[1] - cell_b[1])
    diagonal = min(delta_row, delta_col)
    straight = max(delta_row, delta_col) - diagonal
    return diagonal * SQRT_2 + straight
    # this function was written with the help of ChatGPT at our specific request and editorial remarks


def simplify_grid_path(path):
    """
    Keep the first cell, direction-change cells, and the final cell.

    A* often returns one waypoint every 5 cm. Following every cell would make
    the robot oscillate, so collinear cells are compressed into longer,
    safer waypoint segments.
    """

    if path is None or len(path) <= 2:
        return list(path or [])

    simplified = [path[0]]

    previous_direction = (
        path[1][0] - path[0][0],
        path[1][1] - path[0][1],
    )

    for index in range(1, len(path) - 1):
        current_direction = (
            path[index + 1][0] - path[index][0],
            path[index + 1][1] - path[index][1],
        )

        if current_direction != previous_direction:
            simplified.append(path[index])
            previous_direction = current_direction

    simplified.append(path[-1])
    return simplified
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

