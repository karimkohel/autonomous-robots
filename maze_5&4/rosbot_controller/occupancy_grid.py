"""Evidence-based occupancy grid, frontier clustering, A*, and BMP output.

This is the map/planning data structure from the provided working controller.
Only imports were adjusted for modular use; internal logic is unchanged.
"""

import heapq
import math
import struct
from collections import deque

from config import *
from utils import *

# ============================================================
# Evidence-based occupancy grid
# ============================================================


class OccupancyGrid:
    def __init__(self, size, resolution):
        self.size = size
        self.resolution = resolution
        self.center_row = size // 2
        self.center_col = size // 2

        self.evidence = [
            [0 for _ in range(size)]
            for _ in range(size)
        ]

        # Green floor is a semantic, hard forbidden layer. It is kept
        # separate from LiDAR obstacle evidence so free-space rays cannot
        # erase it.
        self.forbidden_cells = set()

        # Low horizontal walls / overhangs that intersect the robot body
        # height. This is separate from the 2-D LiDAR layer.
        self.clearance_cells = set()

        # Candidate low-clearance cells must be observed in multiple distinct
        # depth frames before becoming permanent obstacles.
        self.clearance_observation_hits = {}

        # Temporary collision-recovery cells use an expiry time instead of
        # becoming permanent low-clearance walls.
        self.temporary_recovery_expiry = {}

        # Centres physically driven through by the robot. These are not raw
        # map evidence; they are used only to soften inflated-only margins and
        # temporary recovery marks in later planning.
        self.traversed_cells = set()

    def is_inside(self, row, col):
        return 0 <= row < self.size and 0 <= col < self.size
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def metric_to_grid(self, x_forward, y_left):
        """
        Convert map-frame metres to image cells.

        Image top  = positive map X.
        Image left = positive map Y.
        """

        row = self.center_row - round(x_forward / self.resolution)
        col = self.center_col - round(y_left / self.resolution)

        if not self.is_inside(row, col):
            return None

        return row, col
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def grid_to_metric(self, row, col):
        """Return the map-frame centre of a grid cell in metres."""

        x_forward = (self.center_row - row) * self.resolution
        y_left = (self.center_col - col) * self.resolution
        return x_forward, y_left
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def update_cell(self, row, col, amount):
        if not self.is_inside(row, col):
            return

        self.evidence[row][col] = clamp(
            self.evidence[row][col] + amount,
            MIN_EVIDENCE,
            MAX_EVIDENCE,
        )
        # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def add_ray(self, origin_x, origin_y, end_x, end_y, endpoint_is_hit):
        origin_cell = self.metric_to_grid(origin_x, origin_y)
        end_cell = self.metric_to_grid(end_x, end_y)

        if origin_cell is None or end_cell is None:
            return

        origin_row, origin_col = origin_cell
        end_row, end_col = end_cell

        ray_cells = bresenham_line(
            origin_col,
            origin_row,
            end_col,
            end_row,
        )

        free_cells = ray_cells[:-1] if endpoint_is_hit else ray_cells

        # Ignore cells immediately around the LiDAR body.
        for col, row in free_cells[3:]:
            self.update_cell(row, col, FREE_UPDATE)

        if endpoint_is_hit:
            self.update_cell(end_row, end_col, OCCUPIED_UPDATE)
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def update_from_point_cloud(self, point_cloud, robot_x, robot_y, theta):
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        lidar_origin_x = robot_x + (
            LIDAR_OFFSET_X * cos_theta
            - LIDAR_OFFSET_Y * sin_theta
        )
        lidar_origin_y = robot_y + (
            LIDAR_OFFSET_X * sin_theta
            + LIDAR_OFFSET_Y * cos_theta
        )

        valid_points = 0

        for point in point_cloud:
            if not (
                math.isfinite(point.x)
                and math.isfinite(point.y)
                and math.isfinite(point.z)
            ):
                continue

            measured_distance = math.hypot(point.x, point.y)

            if measured_distance < MIN_MAPPING_DISTANCE:
                continue

            endpoint_is_hit = measured_distance <= MAX_MAPPING_DISTANCE

            if endpoint_is_hit:
                local_x = LIDAR_OFFSET_X + point.x
                local_y = LIDAR_OFFSET_Y + point.y
            else:
                scale = MAX_MAPPING_DISTANCE / measured_distance
                local_x = LIDAR_OFFSET_X + point.x * scale
                local_y = LIDAR_OFFSET_Y + point.y * scale

            global_x = robot_x + (
                local_x * cos_theta
                - local_y * sin_theta
            )
            global_y = robot_y + (
                local_x * sin_theta
                + local_y * cos_theta
            )

            self.add_ray(
                lidar_origin_x,
                lidar_origin_y,
                global_x,
                global_y,
                endpoint_is_hit,
            )

            valid_points += 1

        return valid_points
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def add_forbidden_metric(self, x_forward, y_left, radius_cells=1):
        """Mark a small disk around a metric point as green forbidden."""

        centre = self.metric_to_grid(x_forward, y_left)
        if centre is None:
            return 0

        centre_row, centre_col = centre
        radius_squared = radius_cells * radius_cells
        newly_added = 0

        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_squared:
                    continue

                row = centre_row + delta_row
                col = centre_col + delta_col

                if not self.is_inside(row, col):
                    continue

                cell = (row, col)
                if cell not in self.forbidden_cells:
                    self.forbidden_cells.add(cell)
                    newly_added += 1

        return newly_added
    # this function was written with the help of ChatGPT at our specific request and editorial remarks


    def add_traversed_metric(self, x_forward, y_left, radius_cells=1):
        """Remember a small disk the robot centre has physically driven through.

        This does not erase raw occupied wall evidence or green forbidden
        cells. It only records that inflated-only cells on this corridor should
        not later close a passage the robot already proved was passable.
        Temporary recovery cells in the same disk are removed immediately.
        """

        centre = self.metric_to_grid(x_forward, y_left)
        if centre is None:
            return 0, 0

        centre_row, centre_col = centre
        radius_squared = radius_cells * radius_cells
        newly_added = 0
        temporary_removed = 0

        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_squared:
                    continue

                row = centre_row + delta_row
                col = centre_col + delta_col

                if not self.is_inside(row, col):
                    continue

                cell = (row, col)
                if cell not in self.traversed_cells:
                    self.traversed_cells.add(cell)
                    newly_added += 1

                if cell in self.temporary_recovery_expiry:
                    del self.temporary_recovery_expiry[cell]
                    temporary_removed += 1

        return newly_added, temporary_removed
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def clear_temporary_recovery_near_metric(self, x_forward, y_left, radius_cells=1):
        """Remove temporary collision marks in a local disk."""

        centre = self.metric_to_grid(x_forward, y_left)
        if centre is None:
            return 0

        centre_row, centre_col = centre
        radius_squared = radius_cells * radius_cells
        removed = 0

        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_squared:
                    continue

                cell = (centre_row + delta_row, centre_col + delta_col)
                if cell in self.temporary_recovery_expiry:
                    del self.temporary_recovery_expiry[cell]
                    removed += 1

        return removed
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def has_lidar_obstacle_near(self, row, col, radius_cells):
        """True when the normal LiDAR map already explains this obstacle."""

        radius_squared = radius_cells * radius_cells

        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_squared:
                    continue

                test_row = row + delta_row
                test_col = col + delta_col

                if (
                    self.is_inside(test_row, test_col)
                    and self.cell_state(test_row, test_col) == "occupied"
                ):
                    return True

        return False
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def confirm_clearance_observations(self, observed_cells):
        """
        Promote low-clearance candidates only after several distinct frames.

        observed_cells is a set, so many neighbouring depth pixels from one
        frame cannot instantly create a permanent wall.
        """

        newly_added = 0

        for cell in observed_cells:
            count = self.clearance_observation_hits.get(cell, 0) + 1
            self.clearance_observation_hits[cell] = min(
                count,
                CLEARANCE_CONFIRMATION_HITS,
            )

            if (
                count >= CLEARANCE_CONFIRMATION_HITS
                and cell not in self.clearance_cells
            ):
                self.clearance_cells.add(cell)
                newly_added += 1

        return newly_added
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def add_clearance_metric(self, x_forward, y_left, radius_cells=1):
        """Mark a small disk as blocked by insufficient vertical clearance."""

        centre = self.metric_to_grid(x_forward, y_left)
        if centre is None:
            return 0

        centre_row, centre_col = centre
        radius_squared = radius_cells * radius_cells
        newly_added = 0

        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_squared:
                    continue

                row = centre_row + delta_row
                col = centre_col + delta_col

                if not self.is_inside(row, col):
                    continue

                cell = (row, col)
                if cell not in self.clearance_cells:
                    self.clearance_cells.add(cell)
                    newly_added += 1

        return newly_added
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def add_temporary_recovery_metric(
        self,
        x_forward,
        y_left,
        current_time,
        radius_cells=1,
        ttl=TEMP_RECOVERY_TTL,
    ):
        """Add a temporary blocked disk used only for collision recovery."""

        centre = self.metric_to_grid(x_forward, y_left)
        if centre is None:
            return 0

        centre_row, centre_col = centre
        radius_squared = radius_cells * radius_cells
        expiry_time = current_time + ttl
        newly_added = 0

        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_squared:
                    continue

                row = centre_row + delta_row
                col = centre_col + delta_col

                if not self.is_inside(row, col):
                    continue

                cell = (row, col)

                if cell not in self.temporary_recovery_expiry:
                    newly_added += 1

                self.temporary_recovery_expiry[cell] = max(
                    expiry_time,
                    self.temporary_recovery_expiry.get(cell, -1.0),
                )

        return newly_added
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def active_temporary_recovery_cells(self, current_time):
        """Return unexpired recovery cells and remove expired entries."""

        expired = [
            cell
            for cell, expiry in self.temporary_recovery_expiry.items()
            if expiry <= current_time
        ]

        for cell in expired:
            del self.temporary_recovery_expiry[cell]

        return set(self.temporary_recovery_expiry)
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def clear_temporary_recovery_cells(self):
        """Remove all temporary collision-recovery exclusions."""

        removed = len(self.temporary_recovery_expiry)
        self.temporary_recovery_expiry.clear()
        return removed
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def inflate_cell_set(self, cells, radius_cells):
        """Return a circular inflation of an arbitrary cell set."""

        inflated = set(cells)
        radius_squared = radius_cells * radius_cells

        offsets = []
        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col <= radius_squared:
                    offsets.append((delta_row, delta_col))

        for base_row, base_col in cells:
            for delta_row, delta_col in offsets:
                row = base_row + delta_row
                col = base_col + delta_col

                if self.is_inside(row, col):
                    inflated.add((row, col))

        return inflated
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def cell_state(self, row, col):
        """Return 'occupied', 'free', or 'unknown'."""

        score = self.evidence[row][col]

        if score >= OCCUPIED_THRESHOLD:
            return "occupied"
        if score <= FREE_THRESHOLD:
            return "free"
        return "unknown"
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def count_classified_cells(self):
        unknown = 0
        free = 0
        occupied = 0

        for row in range(self.size):
            for col in range(self.size):
                state = self.cell_state(row, col)
                if state == "occupied":
                    occupied += 1
                elif state == "free":
                    free += 1
                else:
                    unknown += 1

        return unknown, free, occupied
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    # --------------------------------------------------------
    # Obstacle inflation
    # --------------------------------------------------------

    def build_inflated_obstacles(self, radius_cells):
        """
        Return two sets:
        - occupied_cells: original LiDAR obstacle cells
        - inflated_cells: original obstacles plus their safety margin
        """

        occupied_cells = set()

        for row in range(self.size):
            for col in range(self.size):
                if self.cell_state(row, col) == "occupied":
                    occupied_cells.add((row, col))

        inflated_cells = set(occupied_cells)
        radius_squared = radius_cells * radius_cells

        offsets = []
        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col <= radius_squared:
                    offsets.append((delta_row, delta_col))

        for obstacle_row, obstacle_col in occupied_cells:
            for delta_row, delta_col in offsets:
                row = obstacle_row + delta_row
                col = obstacle_col + delta_col

                if self.is_inside(row, col):
                    inflated_cells.add((row, col))

        return occupied_cells, inflated_cells
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    # --------------------------------------------------------
    # Frontier detection and clustering
    # --------------------------------------------------------

    def detect_frontier_cells(self, inflated_cells):
        """
        A frontier is a known free cell adjacent (4-connected) to at
        least one unknown cell. It must also be outside inflated obstacles.
        """

        frontier_cells = set()
        four_neighbours = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for row in range(1, self.size - 1):
            for col in range(1, self.size - 1):
                cell = (row, col)

                if cell in inflated_cells:
                    continue

                if self.cell_state(row, col) != "free":
                    continue

                touches_unknown = False

                for delta_row, delta_col in four_neighbours:
                    neighbour_row = row + delta_row
                    neighbour_col = col + delta_col

                    if self.cell_state(neighbour_row, neighbour_col) == "unknown":
                        touches_unknown = True
                        break

                if touches_unknown:
                    frontier_cells.add(cell)

        return frontier_cells
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    @staticmethod
    def cluster_frontiers(frontier_cells):
        """Group frontier cells with 8-connected flood fill."""

        remaining = set(frontier_cells)
        clusters = []

        neighbour_offsets = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),            (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ]

        while remaining:
            seed = remaining.pop()
            queue = deque([seed])
            cluster = [seed]

            while queue:
                row, col = queue.popleft()

                for delta_row, delta_col in neighbour_offsets:
                    neighbour = (row + delta_row, col + delta_col)

                    if neighbour in remaining:
                        remaining.remove(neighbour)
                        queue.append(neighbour)
                        cluster.append(neighbour)

            clusters.append(cluster)

        return clusters
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    # --------------------------------------------------------
    # A* planning
    # --------------------------------------------------------

    def traversable(self, cell, inflated_cells, start_cell):
        row, col = cell

        if not self.is_inside(row, col):
            return False

        if euclidean_cells(cell, start_cell) <= ROBOT_START_CLEARANCE_CELLS:
            return True

        if cell in inflated_cells:
            return False

        # Exploration paths are allowed only through observed free space.
        return self.cell_state(row, col) == "free"

    def astar(self, start, goal, inflated_cells):
        """Return (path, cost), or (None, infinity) when unreachable."""

        if start is None or goal is None:
            return None, float("inf")

        if not self.is_inside(*start) or not self.is_inside(*goal):
            return None, float("inf")

        moves = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, SQRT_2),
            (-1, 1, SQRT_2),
            (1, -1, SQRT_2),
            (1, 1, SQRT_2),
        ]

        open_heap = []
        heapq.heappush(open_heap, (octile_heuristic(start, goal), 0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, current_cost, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            if current == goal:
                path = [current]

                while current in came_from:
                    current = came_from[current]
                    path.append(current)

                path.reverse()
                return path, current_cost

            closed.add(current)
            current_row, current_col = current

            for delta_row, delta_col, move_cost in moves:
                neighbour = (
                    current_row + delta_row,
                    current_col + delta_col,
                )

                if not self.traversable(neighbour, inflated_cells, start):
                    continue

                # Prevent diagonal movement through blocked corners.
                if delta_row != 0 and delta_col != 0:
                    side_a = (current_row + delta_row, current_col)
                    side_b = (current_row, current_col + delta_col)

                    if not self.traversable(side_a, inflated_cells, start):
                        continue
                    if not self.traversable(side_b, inflated_cells, start):
                        continue

                tentative_cost = current_cost + move_cost

                if tentative_cost >= g_score.get(neighbour, float("inf")):
                    continue

                came_from[neighbour] = current
                g_score[neighbour] = tentative_cost
                priority = tentative_cost + octile_heuristic(neighbour, goal)
                heapq.heappush(
                    open_heap,
                    (priority, tentative_cost, neighbour),
                )

        return None, float("inf")
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    def choose_frontier_and_path(
        self,
        robot_cell,
        frontier_clusters,
        inflated_cells,
    ):
        """
        Try A* toward useful frontier clusters and return:
        (selected_cluster, goal_cell, path, path_cost, score)
        """

        useful_clusters = [
            cluster
            for cluster in frontier_clusters
            if len(cluster) >= MIN_FRONTIER_CLUSTER_SIZE
        ]

        # For each cluster, choose the frontier cell nearest to the robot as
        # its first A* target. Sort clusters by straight-line proximity.
        candidates = []

        for cluster in useful_clusters:
            candidate_goal = min(
                cluster,
                key=lambda cell: euclidean_cells(robot_cell, cell),
            )

            straight_distance = euclidean_cells(robot_cell, candidate_goal)

            if straight_distance < MIN_FRONTIER_DISTANCE_CELLS:
                continue

            candidates.append((straight_distance, cluster, candidate_goal))

        candidates.sort(key=lambda item: item[0])
        candidates = candidates[:MAX_FRONTIER_CLUSTERS_TO_TEST]

        best_result = None

        for _, cluster, candidate_goal in candidates:
            path, path_cost = self.astar(
                robot_cell,
                candidate_goal,
                inflated_cells,
            )

            if path is None:
                continue

            selection_score = (
                path_cost
                - FRONTIER_SIZE_REWARD * math.sqrt(len(cluster))
            )

            if best_result is None or selection_score < best_result[4]:
                best_result = (
                    cluster,
                    candidate_goal,
                    path,
                    path_cost,
                    selection_score,
                )

        if best_result is None:
            return None, None, None, float("inf"), float("inf")

        return best_result
    # this function was written with the help of ChatGPT at our specific request and editorial remarks

    # --------------------------------------------------------
    # BMP visualization
    # --------------------------------------------------------

    def save_planning_bmp(
        self,
        file_path,
        robot_x,
        robot_y,
        theta,
        travelled_path_points,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
        frontier_cells,
        selected_cluster,
        selected_goal,
        astar_path,
        blue_target_position=None,
        yellow_target_position=None,
    ):
        width = self.size
        height = self.size
        bytes_per_pixel = 3

        unpadded_row_size = width * bytes_per_pixel
        row_padding = (4 - unpadded_row_size % 4) % 4
        padded_row_size = unpadded_row_size + row_padding
        pixel_data_size = padded_row_size * height
        file_size = 54 + pixel_data_size

        file_header = struct.pack(
            "<2sIHHI",
            b"BM",
            file_size,
            0,
            0,
            54,
        )

        information_header = struct.pack(
            "<IiiHHIIiiII",
            40,
            width,
            height,
            1,
            24,
            0,
            pixel_data_size,
            2835,
            2835,
            0,
            0,
        )

        travelled_path_cells = set()
        for path_x, path_y in travelled_path_points:
            cell = self.metric_to_grid(path_x, path_y)
            if cell is not None:
                travelled_path_cells.add(cell)

        robot_cell = self.metric_to_grid(robot_x, robot_y)
        start_cell = self.metric_to_grid(0.0, 0.0)
        astar_path_cells = set(astar_path or [])
        selected_cluster_cells = set(selected_cluster or [])

        blue_target_cell = None
        if blue_target_position is not None:
            blue_target_cell = self.metric_to_grid(
                blue_target_position[0],
                blue_target_position[1],
            )

        yellow_target_cell = None
        if yellow_target_position is not None:
            yellow_target_cell = self.metric_to_grid(
                yellow_target_position[0],
                yellow_target_position[1],
            )

        heading_cells = set()
        for step in range(1, 9):
            distance = step * self.resolution
            heading_x = robot_x + distance * math.cos(theta)
            heading_y = robot_y + distance * math.sin(theta)
            cell = self.metric_to_grid(heading_x, heading_y)
            if cell is not None:
                heading_cells.add(cell)

        with open(file_path, "wb") as image_file:
            image_file.write(file_header)
            image_file.write(information_header)

            for row in range(height - 1, -1, -1):
                row_data = bytearray()

                for col in range(width):
                    cell = (row, col)
                    state = self.cell_state(row, col)

                    if state == "occupied":
                        red, green, blue = 0, 0, 0
                    elif state == "free":
                        red, green, blue = 255, 255, 255
                    else:
                        red, green, blue = 130, 130, 130

                    # Inflated-only safety area: dark grey.
                    if cell in inflated_cells and cell not in occupied_cells:
                        red, green, blue = 70, 70, 70

                    # Inflated margin around green floor: dark green.
                    if (
                        cell in forbidden_inflated_cells
                        and cell not in forbidden_cells
                    ):
                        red, green, blue = 0, 90, 0

                    # Measured green forbidden ground: bright green.
                    if cell in forbidden_cells:
                        red, green, blue = 0, 230, 0

                    # Low-clearance cells are diagnostic-only for the current
                    # Maze 3/4/5 competition run. Do not draw them unless
                    # explicitly enabled, so the BMP reflects the planner's
                    # effective hard/soft obstacle model.
                    if DRAW_LOW_CLEARANCE_CELLS:
                        # Inflated margin around a low-clearance obstacle: brown.
                        if (
                            cell in clearance_inflated_cells
                            and cell not in clearance_cells
                        ):
                            red, green, blue = 95, 55, 20

                        # Measured low-clearance obstacle: bright red-brown.
                        if cell in clearance_cells:
                            red, green, blue = 190, 70, 20

                    # All frontiers: cyan.
                    if cell in frontier_cells:
                        red, green, blue = 0, 220, 220

                    # Selected frontier cluster: purple.
                    if cell in selected_cluster_cells:
                        red, green, blue = 180, 0, 220

                    # Travelled robot path: medium blue.
                    if cell in travelled_path_cells:
                        red, green, blue = 0, 100, 255

                    # Planned A* path: bright yellow.
                    if cell in astar_path_cells:
                        red, green, blue = 255, 230, 0

                    # Selected navigation goal: magenta circle.
                    if selected_goal is not None:
                        goal_row, goal_col = selected_goal
                        if math.hypot(row - goal_row, col - goal_col) <= 2:
                            red, green, blue = 255, 0, 255

                    # Detected blue pillar: dark blue circle.
                    if blue_target_cell is not None:
                        target_row, target_col = blue_target_cell
                        if math.hypot(row - target_row, col - target_col) <= 3:
                            red, green, blue = 0, 0, 255

                    # Detected yellow pillar: gold circle.
                    if yellow_target_cell is not None:
                        target_row, target_col = yellow_target_cell
                        if math.hypot(row - target_row, col - target_col) <= 3:
                            red, green, blue = 255, 215, 0

                    # Initial robot position: light blue.
                    if start_cell is not None:
                        start_row, start_col = start_cell
                        if math.hypot(row - start_row, col - start_col) <= 2:
                            red, green, blue = 80, 180, 255

                    # Current heading: orange.
                    if cell in heading_cells:
                        red, green, blue = 255, 165, 0

                    # Current robot position: red.
                    if robot_cell is not None:
                        robot_row, robot_col = robot_cell
                        if math.hypot(row - robot_row, col - robot_col) <= 3:
                            red, green, blue = 255, 0, 0

                    # BMP uses BGR byte order.
                    row_data.extend([blue, green, red])

                row_data.extend(b"\x00" * row_padding)
                image_file.write(row_data)
    # this function was written with the help of ChatGPT at our specific request and editorial remarks
