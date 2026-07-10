"""Mission runtime and state machine.

This file contains the original mission logic after separating constants,
helpers, occupancy-grid class, and Webots device setup into modules.
The behavior is intended to be identical to the supplied monolithic file.
"""

import math
import os
from statistics import median

from config import *
from utils import *
from occupancy_grid import OccupancyGrid
from robot_io import *

# ============================================================
# Robot control and sensing helpers
# ============================================================


def set_wheel_speeds(left_speed, right_speed):
    global last_command_left, last_command_right

    # Six-times commands are requested, but Webots motors must never receive
    # a value outside the PROTO motor limit.
    left_speed = clamp(
        left_speed,
        -MAX_MOTOR_VELOCITY,
        MAX_MOTOR_VELOCITY,
    )
    right_speed = clamp(
        right_speed,
        -MAX_MOTOR_VELOCITY,
        MAX_MOTOR_VELOCITY,
    )

    last_command_left = left_speed
    last_command_right = right_speed

    for motor in left_motors:
        motor.setVelocity(left_speed)
    for motor in right_motors:
        motor.setVelocity(right_speed)


def stop_robot():
    set_wheel_speeds(0.0, 0.0)


IR_SENSOR_NAMES = ("fl", "fr", "rl", "rr")


def closest_named_ir(ir_values):
    """Return the name and value of the closest IR sensor."""
    index = min(range(len(ir_values)), key=lambda i: ir_values[i])
    return IR_SENSOR_NAMES[index], ir_values[index]


def turn_sweep_ir(ir_values, turn_sign):
    """
    Return the IR reading most relevant to an in-place turn.

    turn_sign > 0: left turn, so front-left and rear-right sweep outward.
    turn_sign < 0: right turn, so front-right and rear-left sweep outward.
    """
    if turn_sign > 0:
        candidates = (("fl", ir_values[0]), ("rr", ir_values[3]))
    else:
        candidates = (("fr", ir_values[1]), ("rl", ir_values[2]))

    return min(candidates, key=lambda item: item[1])


def read_encoders():
    return [encoder.getValue() for encoder in encoders]


def get_compass_heading():
    north = compass.getValues()
    return math.atan2(north[0], north[1])


def point_cloud_clearances(point_cloud):
    front = float("inf")
    left = float("inf")
    right = float("inf")

    for point in point_cloud:
        if not (math.isfinite(point.x) and math.isfinite(point.y)):
            continue

        distance = math.hypot(point.x, point.y)
        if distance < MIN_MAPPING_DISTANCE:
            continue

        angle = math.atan2(point.y, point.x)

        if abs(angle) <= FRONT_HALF_ANGLE:
            front = min(front, distance)
        elif LEFT_MIN_ANGLE <= angle <= LEFT_MAX_ANGLE:
            left = min(left, distance)
        elif RIGHT_MIN_ANGLE <= angle <= RIGHT_MAX_ANGLE:
            right = min(right, distance)

    return front, left, right


def format_distance(value):
    return "inf" if not math.isfinite(value) else f"{value:.2f}"


def analyse_and_map_green_floor():
    """
    Return (green_ratio, newly_added_cells, valid_depth_points).

    Green pixels in the lower central RGB image are paired with the aligned
    depth image. Webots RangeFinder depth is axial depth to the image plane,
    so for horizontal bearing alpha:

        local_x = depth
        local_y = depth * tan(alpha)

    The resulting point is transformed from the robot frame to the global map.
    """

    image = rgb_camera.getImage()
    depth_image = depth_camera.getRangeImage()

    if not image or not depth_image:
        return 0.0, 0, 0

    width = rgb_camera.getWidth()
    height = rgb_camera.getHeight()
    depth_width = depth_camera.getWidth()
    depth_height = depth_camera.getHeight()

    x_start = int(width * 0.12)
    x_end = int(width * 0.88)
    y_start = int(height * 0.58)
    y_end = int(height * 0.98)

    green_pixels = []
    sampled_pixels = 0

    for y in range(y_start, y_end, GREEN_SAMPLE_STEP):
        for x in range(x_start, x_end, GREEN_SAMPLE_STEP):
            red = Camera.imageGetRed(image, width, x, y)
            green = Camera.imageGetGreen(image, width, x, y)
            blue = Camera.imageGetBlue(image, width, x, y)

            sampled_pixels += 1

            if (
                green > 110
                and green > red * 1.45
                and green > blue * 1.45
            ):
                green_pixels.append((x, y))

    if sampled_pixels == 0:
        return 0.0, 0, 0

    ratio = len(green_pixels) / sampled_pixels

    # Do not write isolated colour noise into the semantic map.
    if ratio < GREEN_RATIO_MAP_MINIMUM:
        return ratio, 0, 0

    horizontal_fov = depth_camera.getFov()
    depth_minimum = depth_camera.getMinRange()
    depth_maximum = min(depth_camera.getMaxRange(), GREEN_MAX_DEPTH)

    cos_theta = math.cos(robot_theta)
    sin_theta = math.sin(robot_theta)

    newly_added = 0
    valid_depth_points = 0

    for rgb_x, rgb_y in green_pixels:
        depth_x = int(rgb_x * depth_width / width)
        depth_y = int(rgb_y * depth_height / height)

        depth_x = clamp(depth_x, 0, depth_width - 1)
        depth_y = clamp(depth_y, 0, depth_height - 1)

        depth = depth_image[depth_y * depth_width + depth_x]

        if not math.isfinite(depth):
            continue
        if depth < depth_minimum or depth > depth_maximum:
            continue

        # Pixel left of image centre => positive robot Y (left).
        horizontal_angle = (
            0.5 - (depth_x + 0.5) / depth_width
        ) * horizontal_fov

        local_x = DEPTH_CAMERA_OFFSET_X + depth
        local_y = (
            DEPTH_CAMERA_OFFSET_Y
            + depth * math.tan(horizontal_angle)
        )

        global_x = robot_x + (
            local_x * cos_theta
            - local_y * sin_theta
        )
        global_y = robot_y + (
            local_x * sin_theta
            + local_y * cos_theta
        )

        newly_added += occupancy_grid.add_forbidden_metric(
            global_x,
            global_y,
            GREEN_CORE_RADIUS_CELLS,
        )
        valid_depth_points += 1

    # If the green patch is clearly present but all depth pixels are outside
    # the depth sensor's valid range, place a conservative short rectangle
    # ahead. This prevents the robot from repeatedly choosing the same unsafe
    # route while still leaving room for A* to go around it.
    if ratio >= GREEN_RATIO_REPLAN and valid_depth_points == 0:
        forward = GREEN_FALLBACK_NEAR

        while forward <= GREEN_FALLBACK_FAR + 1e-9:
            lateral = -GREEN_FALLBACK_HALF_WIDTH

            while lateral <= GREEN_FALLBACK_HALF_WIDTH + 1e-9:
                global_x = robot_x + (
                    forward * cos_theta
                    - lateral * sin_theta
                )
                global_y = robot_y + (
                    forward * sin_theta
                    + lateral * cos_theta
                )

                newly_added += occupancy_grid.add_forbidden_metric(
                    global_x,
                    global_y,
                    GREEN_CORE_RADIUS_CELLS,
                )
                lateral += MAP_RESOLUTION

            forward += MAP_RESOLUTION

    return ratio, newly_added, valid_depth_points



def analyse_and_map_low_clearance():
    """
    Classify the nearest central depth surface using its bottom image edge.

    Test 2 calibration:
        surface_y_max <= 0.44 -> PASSABLE_HIGH_WALL
        surface_y_max >= 0.50 -> BLOCKED_LOW_WALL
        otherwise             -> UNCERTAIN

    A blocked wall must be observed in consecutive depth frames before it is
    committed to the hard clearance map. Confirmed surface points are joined
    into a continuous grid barrier so A* cannot pass through gaps between
    sparse sampled pixels.

    Returns:
        (new_confirmed_cells, valid_points, blocking_front_depth)

    blocking_front_depth is finite only for a confirmed blocked low wall.
    """

    global clearance_blocked_frame_count
    global last_clearance_classification
    global last_clearance_surface_y_max
    global last_clearance_surface_width_pixels
    global last_clearance_nearest_depth
    global last_clearance_just_confirmed

    depth_image = depth_camera.getRangeImage()

    if not depth_image:
        clearance_blocked_frame_count = 0
        last_clearance_classification = "NO_DEPTH_IMAGE"
        last_clearance_surface_y_max = None
        last_clearance_surface_width_pixels = 0
        last_clearance_nearest_depth = float("inf")
        last_clearance_just_confirmed = False
        return 0, 0, float("inf")

    width = depth_camera.getWidth()
    height = depth_camera.getHeight()
    horizontal_fov = depth_camera.getFov()

    depth_minimum = max(
        depth_camera.getMinRange(),
        CLEARANCE_MAP_MIN_DEPTH,
    )
    depth_maximum = min(
        depth_camera.getMaxRange(),
        CLEARANCE_MAP_MAX_DEPTH,
    )

    x_start = int(width * CLEARANCE_ROI_X_START)
    x_end = int(width * CLEARANCE_ROI_X_END)
    y_start = int(height * CLEARANCE_ROI_Y_START)
    y_end = int(height * CLEARANCE_ROI_Y_END)

    valid_samples = []

    for y in range(y_start, y_end, CLEARANCE_SAMPLE_STEP):
        for x in range(x_start, x_end, CLEARANCE_SAMPLE_STEP):
            depth = depth_image[y * width + x]

            if not math.isfinite(depth):
                continue

            if depth < depth_minimum or depth > depth_maximum:
                continue

            valid_samples.append((x, y, depth))

    valid_points = len(valid_samples)

    if valid_points < CLEARANCE_MIN_VALID_POINTS:
        clearance_blocked_frame_count = 0
        last_clearance_classification = "CLEAR_OR_TOO_FAR"
        last_clearance_surface_y_max = None
        last_clearance_surface_width_pixels = 0
        last_clearance_nearest_depth = float("inf")
        last_clearance_just_confirmed = False
        return 0, valid_points, float("inf")

    # A robust nearest depth: median of the nearest 10% of valid samples.
    sorted_depths = sorted(sample[2] for sample in valid_samples)
    nearest_count = max(
        1,
        int(math.ceil(len(sorted_depths) * 0.10)),
    )
    nearest_depth = median(sorted_depths[:nearest_count])

    nearest_surface = [
        sample
        for sample in valid_samples
        if sample[2]
        <= nearest_depth + CLEARANCE_NEAREST_SURFACE_MARGIN
    ]

    if len(nearest_surface) < CLEARANCE_MIN_VALID_POINTS:
        clearance_blocked_frame_count = 0
        last_clearance_classification = "INSUFFICIENT_SURFACE"
        last_clearance_surface_y_max = None
        last_clearance_surface_width_pixels = 0
        last_clearance_nearest_depth = nearest_depth
        last_clearance_just_confirmed = False
        return 0, valid_points, float("inf")

    surface_x_values = [sample[0] for sample in nearest_surface]
    surface_y_values = [sample[1] for sample in nearest_surface]

    surface_width_pixels = (
        max(surface_x_values) - min(surface_x_values)
    )
    surface_y_max = max(surface_y_values) / height

    last_clearance_surface_y_max = surface_y_max
    last_clearance_surface_width_pixels = surface_width_pixels
    last_clearance_nearest_depth = nearest_depth

    if surface_width_pixels < CLEARANCE_MIN_SURFACE_WIDTH_PIXELS:
        clearance_blocked_frame_count = 0
        last_clearance_classification = "SURFACE_TOO_NARROW"
        last_clearance_just_confirmed = False
        return 0, valid_points, float("inf")

    if surface_y_max >= CLEARANCE_BLOCKED_BOTTOM_EDGE:
        classification = "BLOCKED_LOW_WALL"
    elif surface_y_max <= CLEARANCE_PASSABLE_BOTTOM_EDGE:
        classification = "PASSABLE_HIGH_WALL"
    else:
        classification = "UNCERTAIN"

    last_clearance_classification = classification
    last_clearance_just_confirmed = False

    if classification == "BLOCKED_LOW_WALL":
        previous_count = clearance_blocked_frame_count
        clearance_blocked_frame_count = min(
            clearance_blocked_frame_count + 1,
            CLEARANCE_CONFIRMATION_HITS,
        )
        last_clearance_just_confirmed = (
            previous_count < CLEARANCE_CONFIRMATION_HITS
            and clearance_blocked_frame_count
            >= CLEARANCE_CONFIRMATION_HITS
        )
    else:
        clearance_blocked_frame_count = 0

    # Do not make a permanent obstacle until the blocked classification is
    # confirmed in consecutive frames.
    if (
        classification != "BLOCKED_LOW_WALL"
        or clearance_blocked_frame_count
        < CLEARANCE_CONFIRMATION_HITS
    ):
        return 0, valid_points, float("inf")

    cos_theta = math.cos(robot_theta)
    sin_theta = math.sin(robot_theta)

    projected_samples = []

    for x, _y, depth in nearest_surface:
        horizontal_angle = (
            0.5 - (x + 0.5) / width
        ) * horizontal_fov

        local_x = DEPTH_CAMERA_OFFSET_X + depth
        local_y = (
            DEPTH_CAMERA_OFFSET_Y
            + depth * math.tan(horizontal_angle)
        )

        global_x = robot_x + (
            local_x * cos_theta
            - local_y * sin_theta
        )
        global_y = robot_y + (
            local_x * sin_theta
            + local_y * cos_theta
        )

        cell = occupancy_grid.metric_to_grid(global_x, global_y)

        if cell is None:
            continue

        projected_samples.append((x, cell))

    if not projected_samples:
        return 0, valid_points, nearest_depth

    # Join neighbouring projected samples to form a continuous barrier.
    projected_samples.sort(key=lambda item: item[0])
    continuous_cells = set()

    previous_cell = None

    for _x, cell in projected_samples:
        continuous_cells.add(cell)

        if previous_cell is not None:
            previous_row, previous_col = previous_cell
            row, col = cell

            for line_col, line_row in bresenham_line(
                previous_col,
                previous_row,
                col,
                row,
            ):
                if occupancy_grid.is_inside(line_row, line_col):
                    continuous_cells.add((line_row, line_col))

        previous_cell = cell

    newly_added = 0

    for row, col in continuous_cells:
        # When LiDAR already explains this location, the normal occupied layer
        # is sufficient. Otherwise, add it to the low-clearance layer.
        if occupancy_grid.has_lidar_obstacle_near(
            row,
            col,
            CLEARANCE_SKIP_LIDAR_RADIUS_CELLS,
        ):
            continue

        metric_x, metric_y = occupancy_grid.grid_to_metric(row, col)

        newly_added += occupancy_grid.add_clearance_metric(
            metric_x,
            metric_y,
            CLEARANCE_CORE_RADIUS_CELLS,
        )

    return newly_added, valid_points, nearest_depth


def target_colour_match(red, green, blue, colour_name):
    """Return True when one RGB pixel matches the requested pillar colour."""

    if colour_name == "blue":
        return (
            blue >= 105
            and blue >= red * 1.50
            and blue >= green * 1.30
            and blue - red >= 45
        )

    if colour_name == "yellow":
        return (
            red >= 165
            and green >= 155
            and blue <= 90
            and abs(red - green) <= 65
            and red >= blue * 2.00
            and green >= blue * 1.85
        )

    return False


def nearest_valid_depth(depth_image, width, height, x, y, minimum, maximum):
    """Return a valid depth near one image pixel, or None."""

    candidates = []

    for radius in (0, 1, 2):
        for delta_y in range(-radius, radius + 1):
            for delta_x in range(-radius, radius + 1):
                px = x + delta_x
                py = y + delta_y

                if not (0 <= px < width and 0 <= py < height):
                    continue

                value = depth_image[py * width + px]

                if (
                    math.isfinite(value)
                    and minimum <= value <= maximum
                ):
                    candidates.append(value)

        if candidates:
            return median(candidates)

    return None


def target_detection_ratio_threshold(colour_name):
    if colour_name == "blue":
        return BLUE_DETECTION_MIN_RATIO
    if colour_name == "yellow":
        return YELLOW_DETECTION_MIN_RATIO
    return 1.0


def target_reached_ratio_threshold(colour_name):
    if colour_name == "blue":
        return BLUE_REACHED_COLOR_RATIO
    if colour_name == "yellow":
        return YELLOW_REACHED_COLOR_RATIO
    return 1.0


def store_target_ratio_diagnostic(
    colour_name,
    *,
    ratio=0.0,
    samples=0,
    sampled_pixels=0,
    bbox_width=0,
    bbox_height=0,
    bbox_height_ratio=0.0,
    centre_x=None,
    centre_y=None,
    depth=float("inf"),
    position_reliable=False,
    accepted=False,
    failure_reason="not_processed",
):
    """
    Save the latest raw RGB measurement even when it is rejected.

    This is deliberately separate from the accepted detection dictionary.
    It allows the console to show ratios below the detection/reached
    thresholds, which is necessary for calibration.
    """

    target_ratio_diagnostics[colour_name] = {
        "colour": colour_name,
        "ratio": ratio,
        "samples": samples,
        "sampled_pixels": sampled_pixels,
        "bbox_width": bbox_width,
        "bbox_height": bbox_height,
        "bbox_height_ratio": bbox_height_ratio,
        "centre_x": centre_x,
        "centre_y": centre_y,
        "depth": depth,
        "position_reliable": position_reliable,
        "accepted": accepted,
        "failure_reason": failure_reason,
    }


def detect_coloured_pillar(colour_name):
    """
    Detect a pillar primarily from RGB colour ratio.

    The latest raw ratio is always stored in target_ratio_diagnostics,
    including frames that fail the minimum sample, ratio, or bounding-box
    requirements.

    Depth is optional and is used only for global target-position estimation.
    """

    image = rgb_camera.getImage()

    if not image:
        store_target_ratio_diagnostic(
            colour_name,
            failure_reason="RGB image unavailable",
        )
        return None

    depth_image = depth_camera.getRangeImage()

    width = rgb_camera.getWidth()
    height = rgb_camera.getHeight()
    depth_width = depth_camera.getWidth()
    depth_height = depth_camera.getHeight()

    x_start = int(width * 0.04)
    x_end = int(width * 0.96)
    y_start = int(height * 0.08)
    y_end = int(height * 0.92)

    colour_pixels = []
    sampled_pixels = 0

    for y in range(y_start, y_end, TARGET_SAMPLE_STEP):
        for x in range(x_start, x_end, TARGET_SAMPLE_STEP):
            red = Camera.imageGetRed(image, width, x, y)
            green = Camera.imageGetGreen(image, width, x, y)
            blue = Camera.imageGetBlue(image, width, x, y)

            sampled_pixels += 1

            if target_colour_match(
                red,
                green,
                blue,
                colour_name,
            ):
                colour_pixels.append((x, y))

    if sampled_pixels == 0:
        store_target_ratio_diagnostic(
            colour_name,
            failure_reason="no RGB samples",
        )
        return None

    colour_ratio = len(colour_pixels) / sampled_pixels

    centre_x = None
    centre_y = None
    bounding_width = 0
    bounding_height = 0
    bbox_height_ratio = 0.0

    if colour_pixels:
        x_values = [pixel[0] for pixel in colour_pixels]
        y_values = [pixel[1] for pixel in colour_pixels]

        centre_x = int(median(x_values))
        centre_y = int(median(y_values))
        bounding_width = max(x_values) - min(x_values)
        bounding_height = max(y_values) - min(y_values)
        bbox_height_ratio = bounding_height / height

    if len(colour_pixels) < TARGET_MIN_SAMPLES:
        store_target_ratio_diagnostic(
            colour_name,
            ratio=colour_ratio,
            samples=len(colour_pixels),
            sampled_pixels=sampled_pixels,
            bbox_width=bounding_width,
            bbox_height=bounding_height,
            bbox_height_ratio=bbox_height_ratio,
            centre_x=centre_x,
            centre_y=centre_y,
            failure_reason=(
                f"colour samples below minimum "
                f"({len(colour_pixels)}/{TARGET_MIN_SAMPLES})"
            ),
        )
        return None

    detection_ratio_threshold = target_detection_ratio_threshold(
        colour_name
    )

    if colour_ratio < detection_ratio_threshold:
        store_target_ratio_diagnostic(
            colour_name,
            ratio=colour_ratio,
            samples=len(colour_pixels),
            sampled_pixels=sampled_pixels,
            bbox_width=bounding_width,
            bbox_height=bounding_height,
            bbox_height_ratio=bbox_height_ratio,
            centre_x=centre_x,
            centre_y=centre_y,
            failure_reason=(
                f"ratio below detection threshold "
                f"({colour_ratio:.4f}/{detection_ratio_threshold:.4f})"
            ),
        )
        return None

    if (
        bounding_width < TARGET_SAMPLE_STEP
        or bounding_height < TARGET_SAMPLE_STEP
    ):
        store_target_ratio_diagnostic(
            colour_name,
            ratio=colour_ratio,
            samples=len(colour_pixels),
            sampled_pixels=sampled_pixels,
            bbox_width=bounding_width,
            bbox_height=bounding_height,
            bbox_height_ratio=bbox_height_ratio,
            centre_x=centre_x,
            centre_y=centre_y,
            failure_reason=(
                f"bounding box too small "
                f"({bounding_width}x{bounding_height})"
            ),
        )
        return None

    valid_depths = []

    if depth_image:
        depth_minimum = depth_camera.getMinRange()
        depth_maximum = min(
            depth_camera.getMaxRange(),
            TARGET_MAX_DEPTH,
        )

        for rgb_x, rgb_y in colour_pixels:
            depth_x = int(rgb_x * depth_width / width)
            depth_y = int(rgb_y * depth_height / height)

            depth_x = int(clamp(
                depth_x,
                0,
                depth_width - 1,
            ))
            depth_y = int(clamp(
                depth_y,
                0,
                depth_height - 1,
            ))

            value = nearest_valid_depth(
                depth_image,
                depth_width,
                depth_height,
                depth_x,
                depth_y,
                depth_minimum,
                depth_maximum,
            )

            if value is not None:
                valid_depths.append(value)

    minimum_depth_samples = max(
        4,
        TARGET_MIN_SAMPLES // 3,
    )
    position_reliable = (
        len(valid_depths) >= minimum_depth_samples
    )

    target_depth = (
        median(valid_depths)
        if position_reliable
        else float("inf")
    )

    horizontal_fov = depth_camera.getFov()
    depth_centre_x = centre_x * depth_width / width

    horizontal_angle = (
        0.5 - (depth_centre_x + 0.5) / depth_width
    ) * horizontal_fov

    global_x = None
    global_y = None

    if position_reliable:
        local_x = DEPTH_CAMERA_OFFSET_X + target_depth
        local_y = (
            DEPTH_CAMERA_OFFSET_Y
            + target_depth * math.tan(horizontal_angle)
        )

        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)

        global_x = robot_x + (
            local_x * cos_theta
            - local_y * sin_theta
        )
        global_y = robot_y + (
            local_x * sin_theta
            + local_y * cos_theta
        )

    store_target_ratio_diagnostic(
        colour_name,
        ratio=colour_ratio,
        samples=len(colour_pixels),
        sampled_pixels=sampled_pixels,
        bbox_width=bounding_width,
        bbox_height=bounding_height,
        bbox_height_ratio=bbox_height_ratio,
        centre_x=centre_x,
        centre_y=centre_y,
        depth=target_depth,
        position_reliable=position_reliable,
        accepted=True,
        failure_reason="accepted detection",
    )

    return {
        "colour": colour_name,
        "global_x": global_x,
        "global_y": global_y,
        "depth": target_depth,
        "position_reliable": position_reliable,
        "bearing": horizontal_angle,
        "samples": len(colour_pixels),
        "ratio": colour_ratio,
        "centre_x": centre_x,
        "centre_y": centre_y,
        "bbox_width": bounding_width,
        "bbox_height": bounding_height,
        "bbox_height_ratio": bbox_height_ratio,
    }


def target_visually_reached(detection, colour_name):
    """
    Confirm arrival using visual colour coverage, not camera depth.
    """

    if detection is None:
        return False

    return (
        detection["ratio"]
        >= target_reached_ratio_threshold(colour_name)
        and detection["bbox_height_ratio"]
        >= TARGET_REACHED_MIN_BBOX_HEIGHT_RATIO
    )


def format_target_ratio_console_line(colour_name):
    """
    Build a detailed one-line diagnostic for the most recent RGB frame.
    """

    diagnostic = target_ratio_diagnostics[colour_name]

    ratio = diagnostic["ratio"]
    bbox_height_ratio = diagnostic["bbox_height_ratio"]
    detection_threshold = target_detection_ratio_threshold(colour_name)
    reached_threshold = target_reached_ratio_threshold(colour_name)

    detection_pass = (
        diagnostic["accepted"]
        and ratio >= detection_threshold
    )
    reached_ratio_pass = ratio >= reached_threshold
    reached_height_pass = (
        bbox_height_ratio
        >= TARGET_REACHED_MIN_BBOX_HEIGHT_RATIO
    )

    depth_text = (
        f"{diagnostic['depth']:.2f} m"
        if math.isfinite(diagnostic["depth"])
        else "invalid"
    )

    return (
        f"{colour_name.upper()} RGB TRACK | "
        f"ratio={ratio:.4f} | "
        f"detect>={detection_threshold:.4f}:"
        f"{'PASS' if detection_pass else 'FAIL'} | "
        f"reach>={reached_threshold:.4f}:"
        f"{'PASS' if reached_ratio_pass else 'FAIL'} | "
        f"bbox_h={bbox_height_ratio:.3f}/"
        f"{TARGET_REACHED_MIN_BBOX_HEIGHT_RATIO:.3f}:"
        f"{'PASS' if reached_height_pass else 'FAIL'} | "
        f"samples={diagnostic['samples']}/"
        f"{diagnostic['sampled_pixels']} | "
        f"depth={depth_text} | "
        f"accepted={'YES' if diagnostic['accepted'] else 'NO'} | "
        f"reason={diagnostic['failure_reason']}"
    )


# ============================================================
# Runtime state
# ============================================================

occupancy_grid = OccupancyGrid(GRID_SIZE, MAP_RESOLUTION)

controller_directory = os.path.dirname(os.path.abspath(__file__))
final_output_path = os.path.join(
    controller_directory,
    "maze3_mission_final.bmp",
)

latest_planning_output_path = None


def planning_output_path_for(plan_number):
    # Reuse one preview file. Writing hundreds of BMPs and updating the
    # Webots console/filesystem can noticeably slow the simulation.
    return os.path.join(
        controller_directory,
        "maze3_mission_plan_latest.bmp",
    )

controller_start_time = robot.getTime()
last_scan_time = -1.0
last_save_time = -1.0
last_status_time = -1.0
last_green_check_time = -1.0
last_clearance_check_time = -1.0
last_target_check_time = -1.0
last_green_ratio = 0.0
last_green_new_cells = 0
last_green_depth_points = 0
total_green_mapping_events = 0
green_confirmation_count = 0
green_rearm_count = 0
green_replan_armed = True
pending_green_reason = ""
last_clearance_new_cells = 0
last_clearance_valid_points = 0
last_clearance_front = float("inf")
last_clearance_nearest_depth = float("inf")
last_clearance_classification = "NOT_CHECKED"
last_clearance_surface_y_max = None
last_clearance_surface_width_pixels = 0
last_clearance_just_confirmed = False
clearance_blocked_frame_count = 0
total_clearance_mapping_events = 0
last_clearance_log_time = -1.0
last_clearance_direct_replan_time = -1000.0

low_wall_stuck_active = False
low_wall_stuck_anchor_depth = float("inf")
low_wall_stuck_anchor_time = -1.0

semantic_block_confirmation_count = 0
last_semantic_block_cell = None
last_semantic_replan_time = -1000.0

last_target_log_time = -1.0
last_target_verify_log_time = -1.0
last_yellow_cache_log_time = -1.0

# Updated on every RGB target check, even when the candidate is rejected.
target_ratio_diagnostics = {
    "blue": {
        "colour": "blue",
        "ratio": 0.0,
        "samples": 0,
        "sampled_pixels": 0,
        "bbox_width": 0,
        "bbox_height": 0,
        "bbox_height_ratio": 0.0,
        "centre_x": None,
        "centre_y": None,
        "depth": float("inf"),
        "position_reliable": False,
        "accepted": False,
        "failure_reason": "not measured yet",
    },
    "yellow": {
        "colour": "yellow",
        "ratio": 0.0,
        "samples": 0,
        "sampled_pixels": 0,
        "bbox_width": 0,
        "bbox_height": 0,
        "bbox_height_ratio": 0.0,
        "centre_x": None,
        "centre_y": None,
        "depth": float("inf"),
        "position_reliable": False,
        "accepted": False,
        "failure_reason": "not measured yet",
    },
}
target_retry_not_before = {
    "blue": -1.0,
    "yellow": -1.0,
}
target_retry_pose = {
    "blue": None,
    "yellow": None,
}

emergency_ir_confirmation_count = 0

# Last wheel command, used by the no-progress detector.
last_command_left = 0.0
last_command_right = 0.0

# Collision-recovery state.
recovery_reason = ""
recovery_start_x = 0.0
recovery_start_y = 0.0
recovery_turn_start_heading = 0.0
recovery_turn_direction = 1
recovery_turn_flipped = False
recovery_count = 0

# Progress watchdog.
progress_anchor_x = 0.0
progress_anchor_y = 0.0
progress_anchor_heading = 0.0
progress_anchor_time = controller_start_time

# No-frontier fallback scan.
no_frontier_scan_count = 0
scan_previous_heading = 0.0
scan_accumulated_angle = 0.0
scan_start_time = -1.0
scan_reason = ""

# No-frontier safe-exploration state.
no_frontier_safe_explore_count = 0
safe_explore_start_x = 0.0
safe_explore_start_y = 0.0
safe_explore_reason = ""
safe_explore_turn_direction = 1
safe_explore_turn_until = -1.0

previous_encoders = None
initial_heading = None

robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
path_points = [(0.0, 0.0)]

phase = "STARTUP"
phase_entry_time = controller_start_time
segment_start_x = 0.0
segment_start_y = 0.0
desired_heading = 0.0
turn_start_heading = 0.0
turn_direction = 1

last_front_lidar = float("inf")
last_left_lidar = float("inf")
last_right_lidar = float("inf")

# Planned frontier/path-following state.
selected_cluster = None
selected_goal = None
planned_astar_path = None
planned_path_cost = float("inf")
planned_selection_score = float("inf")
planned_waypoints = []
waypoint_index = 0

# Continuous exploration state.
frontiers_reached = 0
planning_attempts = 0
last_replan_reason = ""

# Mission state.
mission_state = "SEARCH_BLUE"
active_plan_type = "FRONTIER"
active_target_colour = None

blue_target_position = None
yellow_target_position = None

# Independent target caches. Yellow is updated even while blue is the active
# mission target, so it can be reused immediately after blue is reached.
target_cache = {
    "blue": {
        "position": None,
        "depth": float("inf"),
        "confirmations": 0,
        "last_seen": -1.0,
    },
    "yellow": {
        "position": None,
        "depth": float("inf"),
        "confirmations": 0,
        "last_seen": -1.0,
    },
}

# Last accepted visual detections are kept separately from the global A*
# position cache. This allows an immediate visual-arrival check to override
# replanning when the robot is already close to the active target.
last_target_detection = {
    "blue": None,
    "yellow": None,
}
last_target_detection_time = {
    "blue": -1.0,
    "yellow": -1.0,
}

tracked_target_position = None
tracked_target_depth = float("inf")
target_confirmation_count = 0
target_reached_confirmation_count = 0
last_target_seen_time = -1.0
last_target_plan_time = -1.0
target_verify_start_time = -1.0

blue_reached_simulation_time = None
yellow_reached_simulation_time = None

finished = False


# ============================================================
# Pose update
# ============================================================


def initialize_pose():
    global previous_encoders
    global initial_heading
    global robot_x, robot_y, robot_theta

    previous_encoders = read_encoders()
    initial_heading = get_compass_heading()

    robot_x = 0.0
    robot_y = 0.0
    robot_theta = 0.0


def update_pose():
    global previous_encoders
    global robot_x, robot_y, robot_theta

    current_encoders = read_encoders()

    deltas = [
        current - previous
        for current, previous in zip(current_encoders, previous_encoders)
    ]
    previous_encoders = current_encoders

    left_delta_angle = (deltas[0] + deltas[2]) / 2.0
    right_delta_angle = (deltas[1] + deltas[3]) / 2.0

    left_distance = left_delta_angle * WHEEL_RADIUS
    right_distance = right_delta_angle * WHEEL_RADIUS
    centre_distance = (left_distance + right_distance) / 2.0

    absolute_heading = get_compass_heading()
    new_theta = normalize_angle(absolute_heading - initial_heading)

    delta_theta = normalize_angle(new_theta - robot_theta)
    middle_theta = normalize_angle(robot_theta + delta_theta / 2.0)

    robot_x += centre_distance * math.cos(middle_theta)
    robot_y += centre_distance * math.sin(middle_theta)
    robot_theta = new_theta

    last_path_x, last_path_y = path_points[-1]
    if math.hypot(robot_x - last_path_x, robot_y - last_path_y) >= 0.02:
        path_points.append((robot_x, robot_y))


# ============================================================
# Phase transitions
# ============================================================


def enter_phase(new_phase, current_time):
    global phase, phase_entry_time
    global segment_start_x, segment_start_y
    global desired_heading, turn_start_heading, turn_direction

    stop_robot()
    phase = new_phase
    phase_entry_time = current_time

    if new_phase in ("DRIVE_1", "DRIVE_2"):
        segment_start_x = robot_x
        segment_start_y = robot_y
        desired_heading = robot_theta

    elif new_phase == "TURN":
        turn_start_heading = robot_theta

        if last_left_lidar >= last_right_lidar:
            turn_direction = 1
        else:
            turn_direction = -1

        direction_name = "left" if turn_direction == 1 else "right"
        print(
            f"Choosing a {direction_name} turn | "
            f"left clearance={format_distance(last_left_lidar)} m | "
            f"right clearance={format_distance(last_right_lidar)} m"
        )


def segment_distance():
    return math.hypot(
        robot_x - segment_start_x,
        robot_y - segment_start_y,
    )


# ============================================================
# Planning and path-following stages
# ============================================================


def build_current_planning_layers(current_time=None):
    """Build all active planning layers, including temporary recovery cells."""

    if current_time is None:
        current_time = robot.getTime()

    occupied_cells, lidar_inflated_cells = (
        occupancy_grid.build_inflated_obstacles(
            INFLATION_RADIUS_CELLS
        )
    )

    forbidden_cells = set(occupancy_grid.forbidden_cells)
    forbidden_inflated_cells = occupancy_grid.inflate_cell_set(
        forbidden_cells,
        GREEN_INFLATION_RADIUS_CELLS,
    )

    permanent_clearance_cells = set(occupancy_grid.clearance_cells)
    permanent_clearance_inflated = occupancy_grid.inflate_cell_set(
        permanent_clearance_cells,
        CLEARANCE_INFLATION_RADIUS_CELLS,
    )

    temporary_recovery_cells = (
        occupancy_grid.active_temporary_recovery_cells(current_time)
    )
    temporary_recovery_inflated = occupancy_grid.inflate_cell_set(
        temporary_recovery_cells,
        TEMP_RECOVERY_INFLATION_RADIUS_CELLS,
    )

    # Return the combined layer so the existing planner and visualizer remain
    # compatible, while retaining the permanent/temporary distinction here.
    clearance_cells = (
        permanent_clearance_cells
        | temporary_recovery_cells
    )
    clearance_inflated_cells = (
        permanent_clearance_inflated
        | temporary_recovery_inflated
    )

    inflated_cells = (
        set(lidar_inflated_cells)
        | forbidden_inflated_cells
        | clearance_inflated_cells
    )

    robot_cell = occupancy_grid.metric_to_grid(robot_x, robot_y)

    if robot_cell is not None:
        robot_row, robot_col = robot_cell

        for delta_row in range(
            -ROBOT_START_CLEARANCE_CELLS,
            ROBOT_START_CLEARANCE_CELLS + 1,
        ):
            for delta_col in range(
                -ROBOT_START_CLEARANCE_CELLS,
                ROBOT_START_CLEARANCE_CELLS + 1,
            ):
                if (
                    delta_row * delta_row + delta_col * delta_col
                    <= ROBOT_START_CLEARANCE_CELLS
                    * ROBOT_START_CLEARANCE_CELLS
                ):
                    cell = (
                        robot_row + delta_row,
                        robot_col + delta_col,
                    )

                    # Never clear green or permanent low-clearance geometry.
                    # Temporary recovery inflation is allowed to be cleared
                    # immediately around the current pose so it cannot trap
                    # the A* start cell.
                    if (
                        cell not in forbidden_inflated_cells
                        and cell not in permanent_clearance_inflated
                    ):
                        inflated_cells.discard(cell)

    return (
        robot_cell,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
    )



def required_target_colour():
    if mission_state in ("SEARCH_BLUE", "NAVIGATE_BLUE"):
        return "blue"
    if mission_state in ("SEARCH_YELLOW", "NAVIGATE_YELLOW"):
        return "yellow"
    return None


def current_target_position():
    colour = required_target_colour()

    if colour == "blue":
        return blue_target_position
    if colour == "yellow":
        return yellow_target_position
    return None


def set_current_target_position(position):
    global blue_target_position, yellow_target_position
    global tracked_target_position

    colour = required_target_colour()

    tracked_target_position = position

    if colour == "blue":
        blue_target_position = position
    elif colour == "yellow":
        yellow_target_position = position


def clear_target_tracker():
    global tracked_target_position
    global tracked_target_depth
    global target_confirmation_count
    global target_reached_confirmation_count
    global last_target_seen_time

    tracked_target_position = None
    tracked_target_depth = float("inf")
    target_confirmation_count = 0
    target_reached_confirmation_count = 0
    last_target_seen_time = -1.0


def update_colour_cache(colour_name, detection, current_time):
    """Update one independent blue/yellow global-position cache."""

    global blue_target_position, yellow_target_position

    track = target_cache[colour_name]

    if detection is None:
        return track["confirmations"] >= TARGET_CONFIRMATIONS_REQUIRED

    # A close target can have a strong colour ratio while depth is invalid.
    # Keep that visual detection for reach verification, but only update the
    # global A* cache when a reliable RGB-D position exists.
    if not detection["position_reliable"]:
        track["last_seen"] = current_time
        return track["confirmations"] >= TARGET_CONFIRMATIONS_REQUIRED

    measured_position = (
        detection["global_x"],
        detection["global_y"],
    )

    if track["position"] is None:
        track["position"] = measured_position
        track["confirmations"] = 1
    else:
        displacement = math.hypot(
            measured_position[0] - track["position"][0],
            measured_position[1] - track["position"][1],
        )

        if displacement > TARGET_TRACK_GATE:
            track["position"] = measured_position
            track["confirmations"] = 1
        else:
            track["position"] = (
                (1.0 - TARGET_TRACK_ALPHA) * track["position"][0]
                + TARGET_TRACK_ALPHA * measured_position[0],
                (1.0 - TARGET_TRACK_ALPHA) * track["position"][1]
                + TARGET_TRACK_ALPHA * measured_position[1],
            )
            track["confirmations"] = min(
                track["confirmations"] + 1,
                TARGET_CONFIRMATIONS_REQUIRED + 3,
            )

    track["depth"] = detection["depth"]
    track["last_seen"] = current_time

    if colour_name == "blue":
        blue_target_position = track["position"]
    else:
        yellow_target_position = track["position"]

    return track["confirmations"] >= TARGET_CONFIRMATIONS_REQUIRED


def sync_required_target_tracker():
    """Copy the required-colour cache into the generic mission tracker."""

    global tracked_target_position
    global tracked_target_depth
    global target_confirmation_count
    global last_target_seen_time

    colour_name = required_target_colour()

    if colour_name is None:
        clear_target_tracker()
        return

    track = target_cache[colour_name]
    tracked_target_position = track["position"]
    tracked_target_depth = track["depth"]
    target_confirmation_count = track["confirmations"]
    last_target_seen_time = track["last_seen"]


def update_target_tracker(detection, current_time):
    """Backward-compatible wrapper for the currently required target."""

    colour_name = required_target_colour()

    if colour_name is None:
        return False

    confirmed = update_colour_cache(
        colour_name,
        detection,
        current_time,
    )
    sync_required_target_tracker()
    return confirmed


def update_all_target_caches(current_time, allow_state_transition=True):
    """
    Detect both pillars, cache their global positions, and optionally react.

    This function is intentionally called before scan/recovery phase returns,
    so yellow can be cached while the robot is searching for blue or while it
    is doing a no-frontier 360-degree scan.

    Returns:
        (action_taken, target_check_updated, current_required_detection)
    """

    global last_target_check_time
    global last_target_log_time
    global last_yellow_cache_log_time
    global last_target_plan_time
    global target_reached_confirmation_count

    if current_time - last_target_check_time < TARGET_CHECK_INTERVAL:
        return False, False, None

    last_target_check_time = current_time

    blue_detection = detect_coloured_pillar("blue")
    yellow_detection = detect_coloured_pillar("yellow")

    if blue_detection is not None:
        last_target_detection["blue"] = blue_detection
        last_target_detection_time["blue"] = current_time

    if yellow_detection is not None:
        last_target_detection["yellow"] = yellow_detection
        last_target_detection_time["yellow"] = current_time

    blue_confirmed = update_colour_cache(
        "blue",
        blue_detection,
        current_time,
    )
    yellow_confirmed = update_colour_cache(
        "yellow",
        yellow_detection,
        current_time,
    )

    sync_required_target_tracker()

    required_colour = required_target_colour()
    if required_colour == "blue":
        current_required_detection = blue_detection
        required_confirmed = blue_confirmed
    elif required_colour == "yellow":
        current_required_detection = yellow_detection
        required_confirmed = yellow_confirmed
    else:
        current_required_detection = None
        required_confirmed = False

    if current_time - last_target_log_time >= TARGET_LOG_INTERVAL:
        last_target_log_time = current_time

        # Print both ratios, even if one or both candidates were rejected.
        print(format_target_ratio_console_line("blue"))
        print(format_target_ratio_console_line("yellow"))

        for colour_name, detection in (
            ("blue", blue_detection),
            ("yellow", yellow_detection),
        ):
            if detection is None:
                continue

            track = target_cache[colour_name]
            cached_text = (
                f"({track['position'][0]:.2f}, "
                f"{track['position'][1]:.2f})"
                if track["position"] is not None
                else "unknown"
            )
            print(
                f"{colour_name.upper()} ACCEPTED TARGET | "
                f"cached={cached_text} | "
                f"confirm={track['confirmations']}/"
                f"{TARGET_CONFIRMATIONS_REQUIRED}"
            )

    if (
        mission_state in ("SEARCH_BLUE", "NAVIGATE_BLUE")
        and yellow_confirmed
        and current_time - last_yellow_cache_log_time >= 3.0
    ):
        last_yellow_cache_log_time = current_time
        yellow_track = target_cache["yellow"]
        print(
            "Yellow cached for later | "
            f"position=({yellow_track['position'][0]:.2f}, "
            f"{yellow_track['position'][1]:.2f}) | "
            f"confirm={yellow_track['confirmations']}"
        )

    # Highest priority: if the active target is already visually reached,
    # do not let low-wall, green, frontier, or target-A* replanning pull the
    # robot away. This is the fix for the near-yellow turning problem.
    if (
        required_colour is not None
        and current_required_detection is not None
        and target_visually_reached(
            current_required_detection,
            required_colour,
        )
    ):
        target_reached_confirmation_count += 1
        print(
            f"IMMEDIATE VISUAL {required_colour.upper()} REACH | "
            f"ratio={current_required_detection['ratio']:.4f} | "
            f"bbox={current_required_detection['bbox_height_ratio']:.3f} | "
            f"confirm={target_reached_confirmation_count}/"
            f"{TARGET_REACHED_CONFIRMATIONS_REQUIRED}"
        )

        if (
            target_reached_confirmation_count
            >= TARGET_REACHED_CONFIRMATIONS_REQUIRED
        ):
            handle_target_reached(current_time)
            return True, True, current_required_detection
    else:
        target_reached_confirmation_count = 0

    if not allow_state_transition:
        return False, True, current_required_detection

    if (
        required_confirmed
        and mission_state in ("SEARCH_BLUE", "SEARCH_YELLOW")
        and phase not in (
            "GREEN_OBSERVE",
            "REPLAN",
            "VERIFY_TARGET",
            "RECOVERY_BACKUP",
            "RECOVERY_TURN",
        )
        and current_time - last_target_plan_time
        >= TARGET_REPLAN_COOLDOWN
        and target_retry_allowed(required_colour, current_time)
    ):
        if required_colour == "blue":
            set_mission_state("NAVIGATE_BLUE")
        else:
            set_mission_state("NAVIGATE_YELLOW")

        # Do not jump directly into target planning on the same frame.
        # Stop briefly so green and low-clearance layers can accumulate.
        if last_green_ratio >= GREEN_RATIO_REPLAN:
            enter_green_observation(
                f"{required_colour} found, but green is visible ahead "
                f"(ratio={last_green_ratio:.3f})",
                current_time,
            )
        else:
            request_replan(
                f"{required_colour} pillar confirmed; "
                "observe semantic obstacles then plan",
                current_time,
            )

        return True, True, current_required_detection

    return False, True, current_required_detection


def target_line_of_sight(candidate, target_cell, blocked_cells):
    """Check that the candidate has a mostly clear view toward the pillar."""

    if candidate is None or target_cell is None:
        return False

    candidate_row, candidate_col = candidate
    target_row, target_col = target_cell

    cells = bresenham_line(
        candidate_col,
        candidate_row,
        target_col,
        target_row,
    )

    # Ignore the final cells because the pillar itself is expected to be
    # occupied/inflated in the LiDAR map.
    visible_cells = cells[:-TARGET_LINE_OF_SIGHT_MARGIN_CELLS]

    for col, row in visible_cells[1:]:
        if (row, col) in blocked_cells:
            return False

    return True


def choose_target_approach(
    target_position,
    robot_cell,
    inflated_cells,
):
    """
    Search a ring of safe standoff cells around the detected pillar.

    Returns:
        (approach_cell, astar_path, astar_cost)
    """

    if target_position is None or robot_cell is None:
        return None, None, float("inf")

    target_x, target_y = target_position
    target_cell = occupancy_grid.metric_to_grid(target_x, target_y)

    if target_cell is None:
        return None, None, float("inf")

    best = None
    radius = TARGET_APPROACH_MIN_RADIUS

    while radius <= TARGET_APPROACH_MAX_RADIUS + 1e-9:
        angle = 0.0

        while angle < 2.0 * math.pi - 1e-9:
            candidate_x = target_x + radius * math.cos(angle)
            candidate_y = target_y + radius * math.sin(angle)
            candidate_cell = occupancy_grid.metric_to_grid(
                candidate_x,
                candidate_y,
            )

            angle += TARGET_APPROACH_ANGLE_STEP

            if candidate_cell is None:
                continue

            if candidate_cell in inflated_cells:
                continue

            if occupancy_grid.cell_state(*candidate_cell) != "free":
                continue

            if not target_line_of_sight(
                candidate_cell,
                target_cell,
                inflated_cells,
            ):
                continue

            path, path_cost = occupancy_grid.astar(
                robot_cell,
                candidate_cell,
                inflated_cells,
            )

            if path is None:
                continue

            # Prefer short paths and a standoff close to the configured
            # target distance.
            standoff_penalty = (
                abs(radius - TARGET_PREFERRED_STANDOFF)
                / MAP_RESOLUTION
            )
            score = path_cost + 0.35 * standoff_penalty

            if best is None or score < best[0]:
                best = (
                    score,
                    candidate_cell,
                    path,
                    path_cost,
                )

        radius += TARGET_APPROACH_RADIUS_STEP

    if best is None:
        return None, None, float("inf")

    return best[1], best[2], best[3]


def handle_completed_plan(current_time):
    """Dispatch completion according to the active plan type."""

    global phase, phase_entry_time
    global target_verify_start_time
    global target_reached_confirmation_count

    if active_plan_type == "TARGET":
        stop_robot()
        phase = "VERIFY_TARGET"
        phase_entry_time = current_time
        target_verify_start_time = current_time
        target_reached_confirmation_count = 0

        print("\n----------------------------------------")
        print("TARGET APPROACH POINT REACHED")
        print(f"Mission state: {mission_state}")
        print("Verifying target distance and visibility...")
        print("----------------------------------------\n")
        return

    handle_frontier_reached(current_time)



def target_retry_allowed(colour_name, current_time):
    """Avoid repeatedly attempting an unchanged, unreachable target plan."""

    if colour_name is None:
        return False

    retry_pose = target_retry_pose[colour_name]

    if current_time >= target_retry_not_before[colour_name]:
        return True

    if retry_pose is None:
        return False

    moved = math.hypot(
        robot_x - retry_pose[0],
        robot_y - retry_pose[1],
    )

    return moved >= TARGET_FAILED_RETRY_MOVE


def postpone_target_retry(colour_name, current_time):
    target_retry_not_before[colour_name] = (
        current_time + TARGET_FAILED_RETRY_DELAY
    )
    target_retry_pose[colour_name] = (robot_x, robot_y)


def plan_to_current_target(reason, current_time):
    """Plan an A* route to a safe point near the required pillar."""

    global selected_cluster, selected_goal
    global planned_astar_path, planned_path_cost
    global planned_selection_score, planned_waypoints
    global waypoint_index, phase, phase_entry_time
    global planning_attempts, latest_planning_output_path
    global active_plan_type, active_target_colour
    global no_frontier_scan_count
    global no_frontier_safe_explore_count
    global no_frontier_safe_explore_count
    global last_target_plan_time
    global target_confirmation_count

    target_position = current_target_position()
    target_colour = required_target_colour()
    last_target_plan_time = current_time

    if target_position is None or target_colour is None:
        plan_and_start_following(
            reason + " | target position unavailable; exploring",
            current_time,
        )
        return

    stop_robot()
    planning_attempts += 1

    if planning_attempts > MAX_TOTAL_PLANNING_ATTEMPTS:
        finish_run("maximum planning attempts reached")
        return

    (
        robot_cell,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
    ) = build_current_planning_layers()

    approach_cell, path, path_cost = choose_target_approach(
        target_position,
        robot_cell,
        inflated_cells,
    )

    if path is None:
        print(
            f"No reachable {target_colour} approach cell yet. "
            f"Exploring before retrying for at least "
            f"{TARGET_FAILED_RETRY_DELAY:.1f} s or "
            f"{TARGET_FAILED_RETRY_MOVE:.2f} m."
        )

        postpone_target_retry(target_colour, current_time)
        target_confirmation_count = 0

        if target_colour == "blue":
            set_mission_state("SEARCH_BLUE")
        else:
            set_mission_state("SEARCH_YELLOW")

        plan_and_start_following(
            reason + " | target approach unavailable; explore first",
            current_time,
        )
        return

    selected_cluster = None
    selected_goal = approach_cell
    planned_astar_path = path
    planned_path_cost = path_cost
    planned_selection_score = path_cost

    no_frontier_scan_count = 0
    no_frontier_safe_explore_count = 0

    simplified_cells = simplify_grid_path(planned_astar_path)
    planned_waypoints = [
        occupancy_grid.grid_to_metric(row, col)
        for row, col in simplified_cells
    ]

    while planned_waypoints:
        first_x, first_y = planned_waypoints[0]

        if math.hypot(first_x - robot_x, first_y - robot_y) > WAYPOINT_REACHED_DISTANCE:
            break

        planned_waypoints.pop(0)

    waypoint_index = 0
    active_plan_type = "TARGET"
    active_target_colour = target_colour
    phase = "FOLLOW_PATH"
    phase_entry_time = current_time

    latest_planning_output_path = planning_output_path_for(planning_attempts)

    frontier_cells = occupancy_grid.detect_frontier_cells(inflated_cells)

    occupancy_grid.save_planning_bmp(
        latest_planning_output_path,
        robot_x,
        robot_y,
        robot_theta,
        path_points,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
        frontier_cells,
        None,
        selected_goal,
        planned_astar_path,
        blue_target_position,
        yellow_target_position,
    )

    goal_x, goal_y = occupancy_grid.grid_to_metric(*selected_goal)

    print("\n========================================")
    print("TARGET A* PLAN CREATED")
    print("----------------------------------------")
    print(f"Target colour          : {target_colour}")
    print(f"Mission state          : {mission_state}")
    print(f"Planning trigger       : {reason}")
    print(
        f"Estimated target point : "
        f"x={target_position[0]:.2f}, y={target_position[1]:.2f}"
    )
    print(
        f"Approach point         : "
        f"x={goal_x:.2f}, y={goal_y:.2f}"
    )
    print(
        f"A* path length         : "
        f"{planned_path_cost * MAP_RESOLUTION:.2f} m"
    )
    print(f"Simplified waypoints   : {len(planned_waypoints)}")
    print(f"Planning map           : {latest_planning_output_path}")
    print("========================================\n")

    if not planned_waypoints:
        handle_completed_plan(current_time)
        return

    first_x, first_y = planned_waypoints[0]
    print(
        f"Starting navigation to {target_colour} | "
        f"first waypoint=({first_x:.2f}, {first_y:.2f})"
    )


def plan_for_current_mission(reason, current_time):
    """Choose target navigation or frontier exploration."""

    if mission_state in ("NAVIGATE_BLUE", "NAVIGATE_YELLOW"):
        plan_to_current_target(reason, current_time)
    elif mission_state == "SEARCH_YELLOW":
        yellow_track = target_cache["yellow"]
        cached_yellow_valid = (
            yellow_track["position"] is not None
            and yellow_track["confirmations"]
            >= YELLOW_CACHE_CONFIRMATIONS_AFTER_BLUE
            and current_time - yellow_track["last_seen"]
            <= TARGET_CACHE_MAX_AGE
        )

        if cached_yellow_valid:
            print(
                "SEARCH_YELLOW has cached yellow; switching directly to "
                "NAVIGATE_YELLOW before frontier planning."
            )
            target_retry_not_before["yellow"] = -1.0
            target_retry_pose["yellow"] = None
            set_mission_state("NAVIGATE_YELLOW")
            plan_to_current_target(reason + " | cached yellow available", current_time)
        else:
            plan_and_start_following(reason, current_time)
    else:
        plan_and_start_following(reason, current_time)


def remaining_path_cells():
    """Rasterize the current robot-to-waypoint corridor in map cells."""

    start_cell = occupancy_grid.metric_to_grid(robot_x, robot_y)

    if start_cell is None:
        return set()

    cells = set()
    current_cell = start_cell

    for waypoint_x, waypoint_y in planned_waypoints[waypoint_index:]:
        waypoint_cell = occupancy_grid.metric_to_grid(
            waypoint_x,
            waypoint_y,
        )

        if waypoint_cell is None:
            continue

        current_row, current_col = current_cell
        waypoint_row, waypoint_col = waypoint_cell

        for col, row in bresenham_line(
            current_col,
            current_row,
            waypoint_col,
            waypoint_row,
        ):
            for delta_row in (-1, 0, 1):
                for delta_col in (-1, 0, 1):
                    candidate = (
                        row + delta_row,
                        col + delta_col,
                    )
                    if occupancy_grid.is_inside(*candidate):
                        cells.add(candidate)

        current_cell = waypoint_cell

    return cells


def remaining_path_intersects(blocked_cells):
    corridor = remaining_path_cells()
    intersection = corridor & blocked_cells

    if not intersection:
        return None

    robot_cell = occupancy_grid.metric_to_grid(robot_x, robot_y)

    if robot_cell is None:
        return next(iter(intersection))

    return min(
        intersection,
        key=lambda cell: euclidean_cells(robot_cell, cell),
    )

def print_planning_summary(
    reason,
    occupied_cells,
    inflated_cells,
    frontier_cells,
    frontier_clusters,
):
    unknown, free, occupied = occupancy_grid.count_classified_cells()

    useful_clusters = [
        cluster
        for cluster in frontier_clusters
        if len(cluster) >= MIN_FRONTIER_CLUSTER_SIZE
    ]

    print("\n========================================")
    print("FRONTIER PLAN CREATED")
    print("----------------------------------------")
    print(f"Mission state          : {mission_state}")
    print(f"Frontiers reached      : {frontiers_reached}")
    print(f"Planning attempt: {planning_attempts}/{MAX_TOTAL_PLANNING_ATTEMPTS}")
    print(f"Planning trigger: {reason}")
    print(
        f"Pose: x={robot_x:.3f} m, "
        f"y={robot_y:.3f} m, "
        f"heading={math.degrees(robot_theta):.2f} degrees"
    )
    print()
    print(f"Unknown cells          : {unknown}")
    print(f"Free cells             : {free}")
    print(f"Occupied cells         : {occupied}")
    print(
        f"Green forbidden cells  : "
        f"{len(occupancy_grid.forbidden_cells)}"
    )
    print(
        f"Low-clearance cells    : "
        f"{len(occupancy_grid.clearance_cells)}"
    )
    print(f"Inflated blocked cells : {len(inflated_cells)}")
    print(f"Frontier cells         : {len(frontier_cells)}")
    print(f"All frontier clusters  : {len(frontier_clusters)}")
    print(f"Useful clusters        : {len(useful_clusters)}")

    if planned_astar_path is None:
        print("A* result              : no reachable frontier found")
    else:
        goal_x, goal_y = occupancy_grid.grid_to_metric(*selected_goal)
        print(f"Selected cluster size  : {len(selected_cluster)}")
        print(
            f"Selected goal          : row={selected_goal[0]}, "
            f"col={selected_goal[1]}"
        )
        print(
            f"Goal in map metres     : x={goal_x:.2f}, y={goal_y:.2f}"
        )
        print(f"A* path cells          : {len(planned_astar_path)}")
        print(
            f"A* path length         : "
            f"{planned_path_cost * MAP_RESOLUTION:.2f} m"
        )
        print(f"Simplified waypoints   : {len(planned_waypoints)}")
        print(f"Frontier score         : {planned_selection_score:.2f}")

    print("========================================\n")


def plan_and_start_following(reason, current_time):
    """
    Plan to one reachable frontier, save a visualization, and enter
    FOLLOW_PATH. This function may be called repeatedly.
    """

    global selected_cluster, selected_goal
    global planned_astar_path, planned_path_cost
    global planned_selection_score, planned_waypoints
    global waypoint_index, phase, phase_entry_time
    global planning_attempts, latest_planning_output_path
    global green_confirmation_count
    global emergency_ir_confirmation_count
    global active_plan_type, active_target_colour
    global no_frontier_scan_count
    global no_frontier_safe_explore_count

    stop_robot()

    active_plan_type = "FRONTIER"
    active_target_colour = None
    planning_attempts += 1

    if planning_attempts > MAX_TOTAL_PLANNING_ATTEMPTS:
        finish_run(
            "maximum planning attempts reached before completing "
            f"{MAX_FRONTIERS_TO_VISIT} frontiers"
        )
        return

    (
        robot_cell,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
    ) = build_current_planning_layers()

    frontier_cells = occupancy_grid.detect_frontier_cells(inflated_cells)
    frontier_clusters = occupancy_grid.cluster_frontiers(frontier_cells)

    (
        selected_cluster,
        selected_goal,
        planned_astar_path,
        planned_path_cost,
        planned_selection_score,
    ) = occupancy_grid.choose_frontier_and_path(
        robot_cell,
        frontier_clusters,
        inflated_cells,
    )

    plan_number = planning_attempts
    latest_planning_output_path = planning_output_path_for(plan_number)

    occupancy_grid.save_planning_bmp(
        latest_planning_output_path,
        robot_x,
        robot_y,
        robot_theta,
        path_points,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
        frontier_cells,
        selected_cluster,
        selected_goal,
        planned_astar_path,
        blue_target_position,
        yellow_target_position,
    )

    if planned_astar_path is None:
        print_planning_summary(
            reason,
            occupied_cells,
            inflated_cells,
            frontier_cells,
            frontier_clusters,
        )
        handle_no_reachable_frontier(
            "no reachable frontier was found",
            current_time,
        )
        return

    no_frontier_scan_count = 0
    no_frontier_safe_explore_count = 0

    simplified_cells = simplify_grid_path(planned_astar_path)

    planned_waypoints = [
        occupancy_grid.grid_to_metric(row, col)
        for row, col in simplified_cells
    ]

    # The first A* cell is normally the robot's current cell.
    while planned_waypoints:
        first_x, first_y = planned_waypoints[0]
        if (
            math.hypot(first_x - robot_x, first_y - robot_y)
            > WAYPOINT_REACHED_DISTANCE
        ):
            break
        planned_waypoints.pop(0)

    waypoint_index = 0
    green_confirmation_count = 0
    emergency_ir_confirmation_count = 0

    print_planning_summary(
        reason,
        occupied_cells,
        inflated_cells,
        frontier_cells,
        frontier_clusters,
    )

    print(f"Planning map saved:\n{latest_planning_output_path}")

    if not planned_waypoints:
        request_replan(
            "selected frontier was already within the waypoint tolerance",
            current_time,
        )
        return

    phase = "FOLLOW_PATH"
    phase_entry_time = current_time

    first_x, first_y = planned_waypoints[0]
    print(
        "Starting A* path following | "
        f"frontier={frontiers_reached + 1}/"
        f"{MAX_FRONTIERS_TO_VISIT} | "
        f"first waypoint=({first_x:.2f}, {first_y:.2f}) m"
    )




def set_mission_state(new_state):
    global mission_state
    global active_target_colour

    mission_state = new_state

    if new_state in ("SEARCH_BLUE", "NAVIGATE_BLUE"):
        active_target_colour = "blue"
    elif new_state in ("SEARCH_YELLOW", "NAVIGATE_YELLOW"):
        active_target_colour = "yellow"
    else:
        active_target_colour = None

    sync_required_target_tracker()
    print(f"MISSION STATE -> {mission_state}")


def handle_target_reached(current_time):
    """Record timing and advance from blue to yellow, or finish."""

    global blue_reached_simulation_time
    global yellow_reached_simulation_time
    global target_reached_confirmation_count

    colour = required_target_colour()
    stop_robot()

    if colour == "blue":
        blue_reached_simulation_time = current_time

        print("\n========================================")
        print("BLUE PILLAR REACHED")
        print("----------------------------------------")
        print(
            f"Simulation time start -> blue: "
            f"{blue_reached_simulation_time:.3f} s"
        )
        print(
            f"Pose: x={robot_x:.3f}, y={robot_y:.3f}, "
            f"heading={math.degrees(robot_theta):.2f} deg"
        )
        print("========================================\n")

        clear_target_tracker()
        target_reached_confirmation_count = 0
        set_mission_state("SEARCH_YELLOW")

        yellow_track = target_cache["yellow"]
        yellow_cache_is_valid = (
            yellow_track["position"] is not None
            and yellow_track["confirmations"]
            >= YELLOW_CACHE_CONFIRMATIONS_AFTER_BLUE
            and current_time - yellow_track["last_seen"]
            <= TARGET_CACHE_MAX_AGE
        )

        if yellow_cache_is_valid:
            # Do not let an old failed yellow plan delay the direct cached plan.
            target_retry_not_before["yellow"] = -1.0
            target_retry_pose["yellow"] = None
            set_mission_state("NAVIGATE_YELLOW")
            print(
                "Using cached yellow position observed before blue | "
                f"position=({yellow_track['position'][0]:.2f}, "
                f"{yellow_track['position'][1]:.2f}) | "
                f"age={current_time - yellow_track['last_seen']:.1f} s"
            )
            request_replan(
                "blue reached; navigate directly to cached yellow",
                current_time,
            )
        else:
            print(
                "No reliable cached yellow position yet; "
                "falling back to frontier search."
            )
            request_replan(
                "blue reached; begin searching for yellow",
                current_time,
            )
        return

    if colour == "yellow":
        yellow_reached_simulation_time = current_time

        print("\n========================================")
        print("YELLOW PILLAR REACHED - MISSION SUCCESS")
        print("----------------------------------------")
        print(
            f"Simulation time start -> blue: "
            f"{blue_reached_simulation_time:.3f} s"
        )
        print(
            f"Simulation time blue -> yellow: "
            f"{yellow_reached_simulation_time - blue_reached_simulation_time:.3f} s"
        )
        print(
            f"Total simulation time: "
            f"{yellow_reached_simulation_time:.3f} s"
        )
        print("========================================\n")

        set_mission_state("FINISHED")
        finish_run("blue pillar reached first, then yellow pillar reached")

def enter_green_observation(reason, current_time):
    """
    Stop and observe the same green patch for a short time.

    This lets RGB-D add most of the visible forbidden cells before A*
    is run. The green trigger is then disarmed until the camera view
    becomes clear again, preventing repeated replanning on the same patch.
    """

    global phase, phase_entry_time
    global pending_green_reason
    global green_confirmation_count
    global green_replan_armed
    global emergency_ir_confirmation_count

    stop_robot()

    pending_green_reason = reason
    phase = "GREEN_OBSERVE"
    phase_entry_time = current_time

    green_confirmation_count = 0
    green_replan_armed = False
    emergency_ir_confirmation_count = 0

    print("\n----------------------------------------")
    print("GREEN OBSERVATION STARTED")
    print(f"Reason: {reason}")
    print(
        f"Robot will remain stopped for "
        f"{GREEN_OBSERVE_DURATION:.2f} s"
    )
    print("----------------------------------------\n")


def mark_contact_region(front_ir, current_time):
    """
    Add a hard obstacle patch in front of the robot.

    When the robot physically touches a wall, the current occupancy map may
    still leave a narrow route through that wall because of grid resolution,
    scan height, or pose error. This contact patch prevents A* from returning
    exactly the same escape path.
    """

    measured = front_ir

    if math.isfinite(last_front_lidar):
        measured = min(measured, last_front_lidar)

    # IR is measured from a front-mounted sensor, not from the robot centre.
    # Keep the contact patch a conservative distance in front of the centre.
    mark_distance = clamp(
        measured + 0.17,
        RECOVERY_CONTACT_MARK_MIN_DISTANCE,
        RECOVERY_CONTACT_MARK_MAX_DISTANCE,
    )

    newly_added = 0

    for angle_offset in (
        math.radians(-18.0),
        math.radians(-9.0),
        0.0,
        math.radians(9.0),
        math.radians(18.0),
    ):
        angle = robot_theta + angle_offset
        obstacle_x = robot_x + mark_distance * math.cos(angle)
        obstacle_y = robot_y + mark_distance * math.sin(angle)

        newly_added += occupancy_grid.add_temporary_recovery_metric(
            obstacle_x,
            obstacle_y,
            current_time,
            RECOVERY_CONTACT_MARK_RADIUS_CELLS,
            TEMP_RECOVERY_TTL,
        )

    return newly_added




def mark_contact_region_from_sensors(ir_values, current_time):
    """
    Mark the actual contact direction, not only the robot front.

    The Maze 2/4 failures often happen while turning, where the closest
    contact is front-right, rear-left, or rear-right. The older recovery
    marked only a patch in front of the robot, so A* could choose the same
    side-contact path again and again.
    """

    newly_added = 0

    # Approximate RosBot corner sensor directions in the robot frame.
    # Positive angle is to the robot left.
    sensor_specs = (
        (ir_values[0], math.radians(35.0)),     # front-left
        (ir_values[1], math.radians(-35.0)),    # front-right
        (ir_values[2], math.radians(145.0)),    # rear-left
        (ir_values[3], math.radians(-145.0)),   # rear-right
    )

    for measured, base_angle in sensor_specs:
        if measured > CONTACT_RECOVERY_IR_DISTANCE:
            continue

        mark_distance = clamp(
            measured + 0.16,
            RECOVERY_CONTACT_MARK_MIN_DISTANCE,
            RECOVERY_CONTACT_MARK_MAX_DISTANCE,
        )

        for angle_offset in (
            math.radians(-12.0),
            math.radians(-6.0),
            0.0,
            math.radians(6.0),
            math.radians(12.0),
        ):
            angle = robot_theta + base_angle + angle_offset
            obstacle_x = robot_x + mark_distance * math.cos(angle)
            obstacle_y = robot_y + mark_distance * math.sin(angle)

            newly_added += occupancy_grid.add_temporary_recovery_metric(
                obstacle_x,
                obstacle_y,
                current_time,
                RECOVERY_CONTACT_MARK_RADIUS_CELLS,
                TEMP_RECOVERY_TTL,
            )

    # Also keep the original forward LiDAR fan when the front range agrees.
    front_ir = min(ir_values[0], ir_values[1])
    if (
        front_ir <= CONTACT_RECOVERY_IR_DISTANCE
        or last_front_lidar <= CONTACT_RECOVERY_LIDAR_DISTANCE
        or last_clearance_front <= CONTACT_RECOVERY_LIDAR_DISTANCE
    ):
        newly_added += mark_contact_region(front_ir, current_time)

    return newly_added


def enter_collision_recovery(reason, current_time, ir_values):
    """
    Start a physical escape manoeuvre instead of repeatedly replanning while
    the chassis remains pressed against an obstacle.
    """

    global phase, phase_entry_time
    global recovery_reason
    global recovery_start_x, recovery_start_y
    global recovery_turn_start_heading
    global recovery_turn_direction
    global recovery_turn_flipped
    global recovery_count
    global waypoint_index
    global emergency_ir_confirmation_count
    global progress_anchor_x, progress_anchor_y
    global progress_anchor_heading, progress_anchor_time

    if phase in ("RECOVERY_BACKUP", "RECOVERY_TURN"):
        return

    stop_robot()

    recovery_reason = reason
    recovery_start_x = robot_x
    recovery_start_y = robot_y
    recovery_turn_start_heading = robot_theta
    recovery_turn_flipped = False
    recovery_count += 1

    # Turn away from the closest front corner after backing up.
    # fr closer -> turn left. fl closer -> turn right.
    if ir_values[1] < ir_values[0]:
        recovery_turn_direction = 1
    elif ir_values[0] < ir_values[1]:
        recovery_turn_direction = -1
    elif last_left_lidar > last_right_lidar:
        recovery_turn_direction = 1
    else:
        recovery_turn_direction = -1

    front_ir = min(ir_values[0], ir_values[1])
    contact_confirmed = (
        front_ir <= STUCK_OBSTACLE_CONFIRM_DISTANCE
        or last_front_lidar <= STUCK_OBSTACLE_CONFIRM_DISTANCE
        or last_clearance_front <= STUCK_OBSTACLE_CONFIRM_DISTANCE
        or min(ir_values) <= EMERGENCY_IR_DISTANCE
    )

    newly_marked = 0
    if contact_confirmed:
        newly_marked = mark_contact_region_from_sensors(
            ir_values,
            current_time,
        )

    phase = "RECOVERY_BACKUP"
    phase_entry_time = current_time
    waypoint_index = 0
    emergency_ir_confirmation_count = 0

    progress_anchor_x = robot_x
    progress_anchor_y = robot_y
    progress_anchor_heading = robot_theta
    progress_anchor_time = current_time

    direction_name = (
        "left" if recovery_turn_direction > 0 else "right"
    )

    print("\n========================================")
    print("COLLISION RECOVERY STARTED")
    print("----------------------------------------")
    print(f"Reason: {reason}")
    print(
        f"Front IR: fl={ir_values[0]:.3f} m, "
        f"fr={ir_values[1]:.3f} m"
    )
    print(
        f"Rear IR: rl={ir_values[2]:.3f} m, "
        f"rr={ir_values[3]:.3f} m"
    )
    print(f"Temporary contact cells added: {newly_marked}")
    print(
        f"Recovery plan: reverse {RECOVERY_BACKUP_DISTANCE:.2f} m, "
        f"turn {direction_name} "
        f"{math.degrees(RECOVERY_TURN_ANGLE):.0f} degrees, replan"
    )
    print("========================================\n")


def handle_collision_recovery(current_time, ir_values):
    """Execute the current physical recovery phase."""

    global phase, phase_entry_time
    global recovery_start_x, recovery_start_y
    global recovery_turn_start_heading
    global recovery_turn_direction
    global recovery_turn_flipped
    global progress_anchor_x, progress_anchor_y
    global progress_anchor_heading, progress_anchor_time

    if phase == "RECOVERY_BACKUP":
        rear_ir = min(ir_values[2], ir_values[3])
        backed_distance = math.hypot(
            robot_x - recovery_start_x,
            robot_y - recovery_start_y,
        )
        elapsed = current_time - phase_entry_time

        backup_finished = (
            backed_distance >= RECOVERY_BACKUP_DISTANCE
            or elapsed >= RECOVERY_BACKUP_TIMEOUT
            or rear_ir <= RECOVERY_REAR_STOP_DISTANCE
        )

        if backup_finished:
            stop_robot()
            phase = "RECOVERY_TURN"
            phase_entry_time = current_time
            recovery_turn_start_heading = robot_theta

            reason_parts = []
            if backed_distance >= RECOVERY_BACKUP_DISTANCE:
                reason_parts.append("backup distance reached")
            if elapsed >= RECOVERY_BACKUP_TIMEOUT:
                reason_parts.append("backup timeout")
            if rear_ir <= RECOVERY_REAR_STOP_DISTANCE:
                reason_parts.append(
                    f"rear obstacle {rear_ir:.3f} m"
                )

            print(
                "Recovery backup finished | "
                f"distance={backed_distance:.3f} m | "
                + ", ".join(reason_parts)
            )
            return

        # Reverse along the route that was just traversed.
        set_wheel_speeds(
            -RECOVERY_BACKUP_SPEED,
            -RECOVERY_BACKUP_SPEED,
        )
        return

    if phase == "RECOVERY_TURN":
        elapsed = current_time - phase_entry_time
        turned = abs(
            normalize_angle(
                robot_theta - recovery_turn_start_heading
            )
        )

        turn_sensor_name, turn_ir = turn_sweep_ir(
            ir_values,
            recovery_turn_direction,
        )

        # If the selected turning side is physically blocked, try the other
        # direction once rather than grinding against the same wall.
        if (
            turn_ir <= EMERGENCY_IR_DISTANCE
            and not recovery_turn_flipped
        ):
            recovery_turn_direction *= -1
            recovery_turn_start_heading = robot_theta
            phase_entry_time = current_time
            recovery_turn_flipped = True
            stop_robot()

            print(
                "Recovery turn side blocked | "
                f"{turn_sensor_name}={turn_ir:.3f} m | "
                "trying the opposite direction"
            )
            return

        if (
            turned >= RECOVERY_TURN_ANGLE
            or elapsed >= RECOVERY_TURN_TIMEOUT
        ):
            stop_robot()

            progress_anchor_x = robot_x
            progress_anchor_y = robot_y
            progress_anchor_time = current_time

            print(
                "Collision recovery completed | "
                f"turned={math.degrees(turned):.1f} degrees | "
                f"pose=({robot_x:.3f}, {robot_y:.3f}, "
                f"{math.degrees(robot_theta):.1f} deg)"
            )

            request_replan(
                "physical collision recovery completed | "
                + recovery_reason,
                current_time,
            )
            return

        turn_sign = 1.0 if recovery_turn_direction > 0 else -1.0
        set_wheel_speeds(
            -turn_sign * RECOVERY_TURN_SPEED,
            turn_sign * RECOVERY_TURN_SPEED,
        )




def begin_safe_explore(reason, current_time):
    """Move carefully to open the local map, then return to A*."""

    global phase, phase_entry_time
    global safe_explore_start_x, safe_explore_start_y
    global safe_explore_reason, safe_explore_turn_direction
    global safe_explore_turn_until
    global no_frontier_safe_explore_count
    global green_confirmation_count

    stop_robot()

    no_frontier_safe_explore_count += 1
    safe_explore_start_x = robot_x
    safe_explore_start_y = robot_y
    safe_explore_reason = reason

    # Turn toward the side with more measured free space.
    if last_left_lidar > last_right_lidar:
        safe_explore_turn_direction = 1
    else:
        safe_explore_turn_direction = -1

    safe_explore_turn_until = current_time + SAFE_EXPLORE_INITIAL_TURN
    green_confirmation_count = 0

    phase = "SAFE_EXPLORE"
    phase_entry_time = current_time

    direction_name = "left" if safe_explore_turn_direction > 0 else "right"

    print("\n========================================")
    print("NO-FRONTIER SAFE EXPLORE")
    print("----------------------------------------")
    print(f"Reason: {reason}")
    print(
        f"Attempt: {no_frontier_safe_explore_count}/"
        f"{MAX_NO_FRONTIER_SAFE_EXPLORES}"
    )
    print(
        "A* could not find a frontier, so the robot will make a "
        "short sensor-safe exploration move and then replan."
    )
    print(f"Initial turn: {direction_name}")
    print("========================================\n")


def handle_safe_explore(current_time, front_ir, ir_values, green_check_updated):
    """
    Conservative local exploration used only when frontier A* is stuck.

    This is not the main navigator. It is only a mapper-unblocking motion:
    turn away from green/obstacles, move a short distance, then replan with
    the normal LiDAR + RGB-D + A* stack.
    """

    global safe_explore_turn_until, safe_explore_turn_direction
    global green_confirmation_count

    travelled = math.hypot(
        robot_x - safe_explore_start_x,
        robot_y - safe_explore_start_y,
    )
    elapsed = current_time - phase_entry_time

    if travelled >= SAFE_EXPLORE_DISTANCE or elapsed >= SAFE_EXPLORE_TIMEOUT:
        request_replan(
            safe_explore_reason
            + " | safe exploration completed"
            + f" | moved={travelled:.2f} m",
            current_time,
        )
        return

    # If green is visible, do not keep moving forward. Let the existing
    # RGB-D mapper add forbidden cells, rotate away, then replan.
    if green_check_updated:
        if last_green_ratio >= SAFE_EXPLORE_GREEN_TURN_RATIO:
            green_confirmation_count += 1
        else:
            green_confirmation_count = 0

    if green_confirmation_count >= GREEN_CONFIRMATIONS_REQUIRED:
        safe_explore_turn_direction *= -1
        safe_explore_turn_until = current_time + SAFE_EXPLORE_INITIAL_TURN
        green_confirmation_count = 0

        print(
            "SAFE_EXPLORE: green seen ahead; turning away before "
            f"replanning | ratio={last_green_ratio:.3f}"
        )

    obstacle_close = (
        front_ir <= SAFE_EXPLORE_OBSTACLE_DISTANCE
        or last_front_lidar <= SAFE_EXPLORE_OBSTACLE_DISTANCE
        or last_clearance_front <= SAFE_EXPLORE_OBSTACLE_DISTANCE
    )

    if obstacle_close:
        if last_left_lidar > last_right_lidar:
            safe_explore_turn_direction = 1
        else:
            safe_explore_turn_direction = -1
        safe_explore_turn_until = current_time + SAFE_EXPLORE_INITIAL_TURN

    if current_time <= safe_explore_turn_until:
        turn_sign = 1.0 if safe_explore_turn_direction > 0 else -1.0
        set_wheel_speeds(
            -turn_sign * SAFE_EXPLORE_TURN_SPEED,
            turn_sign * SAFE_EXPLORE_TURN_SPEED,
        )
        return

    # Move slowly and slightly curved, so LiDAR and RGB-D sweep new cells.
    curve = 0.25 * SAFE_EXPLORE_SPEED
    if safe_explore_turn_direction > 0:
        set_wheel_speeds(
            SAFE_EXPLORE_SPEED - curve,
            SAFE_EXPLORE_SPEED + curve,
        )
    else:
        set_wheel_speeds(
            SAFE_EXPLORE_SPEED + curve,
            SAFE_EXPLORE_SPEED - curve,
        )


def begin_no_frontier_scan(reason, current_time):
    """Rotate in place once to refresh the map before declaring failure."""

    global phase, phase_entry_time
    global scan_previous_heading, scan_accumulated_angle
    global scan_start_time, scan_reason

    stop_robot()

    scan_previous_heading = robot_theta
    scan_accumulated_angle = 0.0
    scan_start_time = current_time
    scan_reason = reason

    phase = "SCAN_ROTATE"
    phase_entry_time = current_time

    print("\n========================================")
    print("NO-FRONTIER FALLBACK SCAN")
    print("----------------------------------------")
    print(f"Reason: {reason}")
    print("Temporary recovery cells are already clear.")
    print("The robot will rotate almost 360 degrees and rebuild the map.")
    print("========================================\n")


def handle_no_reachable_frontier(reason, current_time):
    """
    Recover from an apparently closed map.

    1. Remove temporary recovery exclusions and retry.
    2. Rotate once to refresh LiDAR/RGB-D mapping and retry.
    3. Only then terminate if no route exists.
    """

    global no_frontier_scan_count

    active_temporary = occupancy_grid.active_temporary_recovery_cells(
        current_time
    )

    if active_temporary:
        removed = occupancy_grid.clear_temporary_recovery_cells()

        print("\n----------------------------------------")
        print("NO FRONTIER: TEMPORARY CELLS REMOVED")
        print(f"Removed temporary recovery cells: {removed}")
        print("Retrying planning before declaring failure.")
        print("----------------------------------------\n")

        request_replan(
            reason + " | temporary recovery cells removed",
            current_time,
        )
        return

    if no_frontier_scan_count < MAX_CONSECUTIVE_NO_FRONTIER_SCANS:
        begin_no_frontier_scan(reason, current_time)
        return

    if no_frontier_safe_explore_count < MAX_NO_FRONTIER_SAFE_EXPLORES:
        begin_safe_explore(
            reason
            + " | no route after map-refresh scans",
            current_time,
        )
        return

    finish_run(
        reason
        + " | no route after temporary-cell clearing, scans, "
        + "and safe-exploration attempts"
    )


def handle_no_frontier_scan(current_time):
    """Execute the near-360-degree map-refresh rotation."""

    global scan_previous_heading, scan_accumulated_angle
    global no_frontier_scan_count

    delta = abs(
        normalize_angle(robot_theta - scan_previous_heading)
    )
    scan_accumulated_angle += delta
    scan_previous_heading = robot_theta

    elapsed = current_time - scan_start_time

    if (
        scan_accumulated_angle >= NO_FRONTIER_SCAN_TARGET
        or elapsed >= NO_FRONTIER_SCAN_TIMEOUT
    ):
        stop_robot()
        no_frontier_scan_count += 1

        print(
            "No-frontier scan finished | "
            f"rotation={math.degrees(scan_accumulated_angle):.1f} deg | "
            f"elapsed={elapsed:.2f} s"
        )

        request_replan(
            scan_reason + " | map-refresh rotation completed",
            current_time,
        )
        return

    set_wheel_speeds(
        -NO_FRONTIER_SCAN_SPEED,
        NO_FRONTIER_SCAN_SPEED,
    )

def request_replan(reason, current_time):
    """Stop, keep mapping, and request a fresh frontier plan."""

    global phase, phase_entry_time
    global waypoint_index
    global green_confirmation_count
    global green_replan_armed
    global emergency_ir_confirmation_count
    global last_replan_reason
    global progress_anchor_x, progress_anchor_y
    global progress_anchor_heading, progress_anchor_time

    stop_robot()

    last_replan_reason = reason
    phase = "REPLAN"
    phase_entry_time = current_time
    waypoint_index = 0
    green_confirmation_count = 0
    emergency_ir_confirmation_count = 0
    progress_anchor_x = robot_x
    progress_anchor_y = robot_y
    progress_anchor_heading = robot_theta
    progress_anchor_time = current_time

    print("\n----------------------------------------")
    print("REPLAN REQUESTED")
    print(f"Reason: {reason}")
    print(f"Mission state: {mission_state}")
    print(f"Exploration frontiers reached: {frontiers_reached}")
    print("----------------------------------------\n")


def handle_frontier_reached(current_time):
    """Count the reached frontier and either replan or finish."""

    global frontiers_reached

    frontiers_reached += 1

    print("\n========================================")
    print("FRONTIER REACHED")
    print("----------------------------------------")
    print(f"Completed frontier: {frontiers_reached}")
    print(f"Searching for: {required_target_colour()}")
    print(
        f"Pose: x={robot_x:.3f} m, "
        f"y={robot_y:.3f} m, "
        f"heading={math.degrees(robot_theta):.2f} degrees"
    )
    print("========================================\n")

    request_replan(
        f"frontier {frontiers_reached} reached while searching for "
        f"{required_target_colour()}",
        current_time,
    )

def finish_run(reason):
    """Stop once, save the mission map, and write timing results."""

    global finished

    if finished:
        return

    finished = True
    stop_robot()

    (
        robot_cell,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
    ) = build_current_planning_layers()

    frontier_cells = occupancy_grid.detect_frontier_cells(inflated_cells)

    occupancy_grid.save_planning_bmp(
        final_output_path,
        robot_x,
        robot_y,
        robot_theta,
        path_points,
        occupied_cells,
        forbidden_cells,
        forbidden_inflated_cells,
        clearance_cells,
        clearance_inflated_cells,
        inflated_cells,
        frontier_cells,
        selected_cluster,
        selected_goal,
        planned_astar_path,
        blue_target_position,
        yellow_target_position,
    )

    unknown, free, occupied = occupancy_grid.count_classified_cells()

    results_path = os.path.join(
        controller_directory,
        "maze3_mission_timings.txt",
    )

    start_to_blue = (
        blue_reached_simulation_time
        if blue_reached_simulation_time is not None
        else None
    )

    blue_to_yellow = None
    if (
        blue_reached_simulation_time is not None
        and yellow_reached_simulation_time is not None
    ):
        blue_to_yellow = (
            yellow_reached_simulation_time
            - blue_reached_simulation_time
        )

    with open(results_path, "w", encoding="utf-8") as result_file:
        result_file.write("Maze 3 mission result\n")
        result_file.write(f"Reason: {reason}\n")
        result_file.write(f"Mission state: {mission_state}\n")
        result_file.write(
            "Start simulation -> blue pillar reached: "
            + (
                f"{start_to_blue:.3f} s\n"
                if start_to_blue is not None
                else "not reached\n"
            )
        )
        result_file.write(
            "Blue pillar reached -> yellow pillar reached: "
            + (
                f"{blue_to_yellow:.3f} s\n"
                if blue_to_yellow is not None
                else "not reached\n"
            )
        )
        result_file.write(
            "Total simulation time: "
            + (
                f"{yellow_reached_simulation_time:.3f} s\n"
                if yellow_reached_simulation_time is not None
                else "mission incomplete\n"
            )
        )

    print("\n========================================")
    print("BLUE -> YELLOW MISSION FINISHED")
    print("----------------------------------------")
    print(f"Reason: {reason}")
    print(f"Mission state: {mission_state}")
    print(
        f"Final pose: x={robot_x:.3f} m, "
        f"y={robot_y:.3f} m, "
        f"heading={math.degrees(robot_theta):.2f} degrees"
    )
    print(f"Exploration frontiers reached: {frontiers_reached}")
    print(f"Planning attempts: {planning_attempts}")
    print(f"Collision recoveries: {recovery_count}")
    print(
        f"Active temporary recovery cells: "
        f"{len(occupancy_grid.active_temporary_recovery_cells(robot.getTime()))}"
    )
    print(f"Unknown cells : {unknown}")
    print(f"Free cells    : {free}")
    print(f"Occupied cells: {occupied}")
    print(
        f"Green forbidden cells: "
        f"{len(occupancy_grid.forbidden_cells)}"
    )
    print(
        f"Low-clearance cells: "
        f"{len(occupancy_grid.clearance_cells)}"
    )

    if start_to_blue is None:
        print("Start -> blue time: not reached")
    else:
        print(f"Start -> blue time: {start_to_blue:.3f} s")

    if blue_to_yellow is None:
        print("Blue -> yellow time: not reached")
    else:
        print(f"Blue -> yellow time: {blue_to_yellow:.3f} s")

    print("Final mission map:")
    print(final_output_path)
    print("Timing results:")
    print(results_path)
    print("========================================\n")

def follow_planned_path(
    current_time,
    front_ir,
    ir_values,
    green_check_updated,
):
    """Execute one safe waypoint-following control step."""

    global waypoint_index
    global green_confirmation_count
    global emergency_ir_confirmation_count

    if waypoint_index >= len(planned_waypoints):
        handle_completed_plan(current_time)
        return

    target_x, target_y = planned_waypoints[waypoint_index]
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    distance_to_waypoint = math.hypot(delta_x, delta_y)

    final_waypoint = waypoint_index == len(planned_waypoints) - 1
    reached_distance = (
        GOAL_REACHED_DISTANCE
        if final_waypoint
        else WAYPOINT_REACHED_DISTANCE
    )

    if distance_to_waypoint <= reached_distance:
        print(
            f"Waypoint {waypoint_index + 1}/"
            f"{len(planned_waypoints)} reached | "
            f"target=({target_x:.2f}, {target_y:.2f})"
        )
        waypoint_index += 1
        stop_robot()

        if waypoint_index >= len(planned_waypoints):
            handle_completed_plan(current_time)
        return

    target_heading = math.atan2(delta_y, delta_x)
    heading_error = normalize_angle(target_heading - robot_theta)

    # During an in-place turn, only the two corners that sweep outward
    # are relevant. Requiring consecutive detections prevents a single
    # noisy IR sample from stopping the run.
    if abs(heading_error) > TURN_IN_PLACE_THRESHOLD:
        green_confirmation_count = 0

        turn_sign = 1.0 if heading_error > 0.0 else -1.0
        turn_sensor_name, turn_ir = turn_sweep_ir(ir_values, turn_sign)

        if turn_ir < EMERGENCY_IR_DISTANCE:
            emergency_ir_confirmation_count += 1
        else:
            emergency_ir_confirmation_count = 0

        if (
            emergency_ir_confirmation_count
            >= EMERGENCY_CONFIRMATIONS_REQUIRED
        ):
            enter_collision_recovery(
                "confirmed turn-clearance contact while following path "
                f"({turn_sensor_name}={turn_ir:.3f} m)",
                current_time,
                ir_values,
            )
            return

        set_wheel_speeds(
            -turn_sign * PATH_TURN_SPEED,
            turn_sign * PATH_TURN_SPEED,
        )
        return

    # When driving forward, only the front sensors should trigger the
    # contact-level emergency guard. The normal 0.30 m obstacle guard
    # below still handles ordinary obstacle avoidance.
    if front_ir < EMERGENCY_IR_DISTANCE:
        emergency_ir_confirmation_count += 1
    else:
        emergency_ir_confirmation_count = 0

    if (
        emergency_ir_confirmation_count
        >= EMERGENCY_CONFIRMATIONS_REQUIRED
    ):
        enter_collision_recovery(
            f"confirmed front contact while following path "
            f"({front_ir:.3f} m)",
            current_time,
            ir_values,
        )
        return

    obstacle_ahead = (
        front_ir < FRONT_IR_STOP_DISTANCE
        or last_front_lidar < FRONT_LIDAR_STOP_DISTANCE
        or last_clearance_front < CLEARANCE_FRONT_STOP_DISTANCE
    )

    if obstacle_ahead:
        obstacle_reason = (
            "new obstacle detected on the planned path "
            f"(IR={front_ir:.3f} m, "
            f"LiDAR={format_distance(last_front_lidar)} m, "
            f"depth-clearance={format_distance(last_clearance_front)} m)"
        )

        contact_level = (
            front_ir <= CONTACT_RECOVERY_IR_DISTANCE
            or last_front_lidar <= CONTACT_RECOVERY_LIDAR_DISTANCE
        )

        if contact_level:
            enter_collision_recovery(
                obstacle_reason,
                current_time,
                ir_values,
            )
        else:
            request_replan(
                obstacle_reason,
                current_time,
            )
        return

    # Green confirmation must use distinct camera samples. The previous
    # version incremented once per simulation step and could count the
    # same RGB-D frame several times.
    if green_check_updated:
        if (
            green_replan_armed
            and abs(heading_error) <= FORWARD_ALLOWED_HEADING_ERROR
            and last_green_ratio >= GREEN_RATIO_REPLAN
        ):
            green_confirmation_count += 1
        else:
            green_confirmation_count = 0

    if (
        green_replan_armed
        and green_confirmation_count >= GREEN_CONFIRMATIONS_REQUIRED
    ):
        enter_green_observation(
            f"green floor confirmed ahead "
            f"(ratio={last_green_ratio:.3f}, "
            f"new cells={last_green_new_cells}, "
            f"depth points={last_green_depth_points})",
            current_time,
        )
        return

    correction = clamp(
        PATH_HEADING_KP * heading_error,
        -PATH_MAX_HEADING_CORRECTION,
        PATH_MAX_HEADING_CORRECTION,
    )

    # Slow down near a waypoint to reduce overshoot.
    speed_scale = clamp(distance_to_waypoint / 0.22, 0.65, 1.0)

    # A surface whose lower edge lies in the calibrated uncertain band must
    # be observed more carefully instead of entering at full speed.
    if (
        last_clearance_classification == "UNCERTAIN"
        or (
            last_clearance_classification == "BLOCKED_LOW_WALL"
            and clearance_blocked_frame_count
            < CLEARANCE_CONFIRMATION_HITS
        )
    ):
        speed_scale *= CLEARANCE_UNCERTAIN_SPEED_SCALE

    base_speed = PATH_FORWARD_SPEED * speed_scale

    set_wheel_speeds(
        base_speed - correction,
        base_speed + correction,
    )


# ============================================================
# Startup message
# ============================================================

print("\n========================================")
print("MAZE 3 BLUE -> YELLOW AUTONOMOUS MISSION")
print("----------------------------------------")
print("Mission:")
print("1. Detect and reach the blue pillar first")
print("2. Then detect and reach the yellow pillar")
print("3. Never drive over mapped green ground")
print("4. Save the required simulation timings")
print()
print("Navigation stack:")
print("LiDAR mapping + RGB-D green map + frontier exploration + A*")
print("RGB-D targets + confirmed clearance mapping + stable replanning")
print("Yellow is cached even while blue is still the active target")
print("Depth-camera body band blocks low bridges/crossbars missed by 2-D LiDAR")
print("Colour-ratio arrival + calibrated floating-wall blocking + 2x speed")
print("========================================\n")


# ============================================================
# Main loop
# ============================================================

while robot.step(time_step) != -1:
    current_time = robot.getTime()

    if finished:
        stop_robot()
        continue

    elapsed_from_start = current_time - controller_start_time

    if elapsed_from_start < STARTUP_DELAY:
        stop_robot()
        continue

    if phase == "STARTUP":
        initialize_pose()
        enter_phase("DRIVE_1", current_time)
        print("Pose initialized. Starting first forward segment.")
        continue

    update_pose()

    # Updated every loop by update_all_target_caches(). VERIFY_TARGET uses
    # these values to count visual-arrival confirmations.
    target_check_updated = False
    current_target_detection = None

    # --------------------------------------------------------
    # LiDAR mapping and clearances
    # --------------------------------------------------------

    if current_time - last_scan_time >= SCAN_INTERVAL:
        last_scan_time = current_time

        point_cloud = lidar.getPointCloud()

        if point_cloud:
            occupancy_grid.update_from_point_cloud(
                point_cloud,
                robot_x,
                robot_y,
                robot_theta,
            )

            (
                last_front_lidar,
                last_left_lidar,
                last_right_lidar,
            ) = point_cloud_clearances(point_cloud)

    # --------------------------------------------------------
    # Safety readings
    # --------------------------------------------------------

    ir_values = [sensor.getValue() for sensor in range_sensors]
    front_ir = min(ir_values[0], ir_values[1])
    closest_ir_name, closest_ir = closest_named_ir(ir_values)

    green_check_updated = False

    if current_time - last_green_check_time >= GREEN_CHECK_INTERVAL:
        green_check_updated = True
        last_green_check_time = current_time
        (
            last_green_ratio,
            last_green_new_cells,
            last_green_depth_points,
        ) = analyse_and_map_green_floor()

        if last_green_new_cells > 0:
            total_green_mapping_events += 1
            print(
                "Green map update | "
                f"ratio={last_green_ratio:.3f} | "
                f"new cells={last_green_new_cells} | "
                f"depth points={last_green_depth_points} | "
                f"total forbidden="
                f"{len(occupancy_grid.forbidden_cells)}"
            )

    # --------------------------------------------------------
    # RGB-D low-clearance / overhang mapping
    # --------------------------------------------------------

    clearance_check_updated = False

    if (
        current_time - last_clearance_check_time
        >= CLEARANCE_CHECK_INTERVAL
    ):
        clearance_check_updated = True
        last_clearance_check_time = current_time

        (
            last_clearance_new_cells,
            last_clearance_valid_points,
            last_clearance_front,
        ) = analyse_and_map_low_clearance()

        should_log_clearance = (
            last_clearance_classification
            in (
                "BLOCKED_LOW_WALL",
                "UNCERTAIN",
                "PASSABLE_HIGH_WALL",
            )
            and current_time - last_clearance_log_time
            >= CLEARANCE_LOG_INTERVAL
        )

        if last_clearance_new_cells > 0:
            total_clearance_mapping_events += 1

        if should_log_clearance:
            last_clearance_log_time = current_time

            y_max_text = (
                f"{last_clearance_surface_y_max:.3f}"
                if last_clearance_surface_y_max is not None
                else "n/a"
            )

            print(
                "CLEARANCE CLASSIFICATION | "
                f"class={last_clearance_classification} | "
                f"depth="
                f"{format_distance(last_clearance_nearest_depth)} m | "
                f"surface_y_max={y_max_text} | "
                f"width={last_clearance_surface_width_pixels}px | "
                f"confirm={clearance_blocked_frame_count}/"
                f"{CLEARANCE_CONFIRMATION_HITS} | "
                f"new cells={last_clearance_new_cells} | "
                f"total blocked="
                f"{len(occupancy_grid.clearance_cells)}"
            )

        # Stop before entering beneath a confirmed low crossbar. This direct
        # replan happens once at the confirmation transition; afterwards the
        # new continuous clearance barrier is part of every A* plan.
        if (
            phase == "FOLLOW_PATH"
            and last_clearance_just_confirmed
            and last_clearance_front
            <= CLEARANCE_FRONT_REPLAN_DISTANCE
            and current_time - last_clearance_direct_replan_time
            >= SEMANTIC_REPLAN_COOLDOWN
        ):
            (
                action_taken,
                target_check_updated,
                current_target_detection,
            ) = update_all_target_caches(current_time)
            if action_taken:
                continue

            last_clearance_direct_replan_time = current_time
            stop_robot()
            request_replan(
                "confirmed low floating wall blocks robot body "
                f"(depth={last_clearance_front:.2f} m, "
                f"surface_y_max="
                f"{last_clearance_surface_y_max:.3f})",
                current_time,
            )
            continue

    # --------------------------------------------------------
    # Detect/cache both targets before any phase handler can return early.
    # This is critical: during fallback 360-degree scans, yellow can now be
    # cached and used immediately instead of waiting for the full scan to end.
    # --------------------------------------------------------

    if TARGET_SCAN_CACHE_ENABLED:
        (
            action_taken,
            target_check_updated,
            current_target_detection,
        ) = update_all_target_caches(current_time)
        if action_taken:
            continue

    # --------------------------------------------------------
    # Physical collision recovery / no-frontier scan
    # --------------------------------------------------------

    if phase == "SCAN_ROTATE":
        handle_no_frontier_scan(current_time)
        continue

    if phase == "SAFE_EXPLORE":
        handle_safe_explore(
            current_time,
            front_ir,
            ir_values,
            green_check_updated,
        )
        continue

    if phase in ("RECOVERY_BACKUP", "RECOVERY_TURN"):
        handle_collision_recovery(
            current_time,
            ir_values,
        )
        continue

    # A plan may become invalid when a confirmed green/clearance cell is
    # added. Use hysteresis and a cooldown so one physical wall does not cause
    # dozens of nearly identical A* replans.
    if (
        phase == "FOLLOW_PATH"
        and (
            last_green_new_cells > 0
            or last_clearance_new_cells > 0
        )
    ):
        (
            _robot_cell,
            _occupied_cells,
            _forbidden_cells,
            forbidden_inflated_cells,
            _clearance_cells,
            clearance_inflated_cells,
            _inflated_cells,
        ) = build_current_planning_layers()

        semantic_blocked = (
            forbidden_inflated_cells
            | clearance_inflated_cells
        )
        blocked_path_cell = remaining_path_intersects(
            semantic_blocked
        )

        if blocked_path_cell is None:
            semantic_block_confirmation_count = 0
            last_semantic_block_cell = None
        else:
            same_region = (
                last_semantic_block_cell is not None
                and euclidean_cells(
                    blocked_path_cell,
                    last_semantic_block_cell,
                ) <= SEMANTIC_SAME_REGION_RADIUS_CELLS
            )

            if same_region:
                semantic_block_confirmation_count += 1
            else:
                semantic_block_confirmation_count = 1
                last_semantic_block_cell = blocked_path_cell

            cooldown_ready = (
                current_time - last_semantic_replan_time
                >= SEMANTIC_REPLAN_COOLDOWN
            )

            if (
                semantic_block_confirmation_count
                >= SEMANTIC_BLOCK_CONFIRMATIONS_REQUIRED
                and cooldown_ready
            ):
                (
                    action_taken,
                    target_check_updated,
                    current_target_detection,
                ) = update_all_target_caches(current_time)
                if action_taken:
                    continue

                block_x, block_y = occupancy_grid.grid_to_metric(
                    *blocked_path_cell
                )
                last_semantic_replan_time = current_time
                semantic_block_confirmation_count = 0

                request_replan(
                    "confirmed semantic blockage on remaining path at "
                    f"({block_x:.2f}, {block_y:.2f})",
                    current_time,
                )
                continue

    # Target detection/caching was already handled above, before phase handlers
    # and replanning guards, so it can interrupt scans and avoid late target
    # replans when the active pillar is visually reached.

    # Re-arm the green trigger only after several genuinely new RGB-D
    # samples show that the robot is no longer facing the same patch.
    if green_check_updated and not green_replan_armed:
        if last_green_ratio <= GREEN_REARM_RATIO:
            green_rearm_count += 1
        else:
            green_rearm_count = 0

        if green_rearm_count >= GREEN_REARM_CONFIRMATIONS_REQUIRED:
            green_replan_armed = True
            green_rearm_count = 0
            print(
                "Green trigger re-armed | "
                f"ratio={last_green_ratio:.3f}"
            )

    if phase in ("DRIVE_1", "DRIVE_2") and green_check_updated:
        if (
            green_replan_armed
            and last_green_ratio >= GREEN_RATIO_REPLAN
        ):
            green_confirmation_count += 1
        else:
            green_confirmation_count = 0

        if (
            green_replan_armed
            and green_confirmation_count
            >= GREEN_CONFIRMATIONS_REQUIRED
        ):
            enter_green_observation(
                f"green floor confirmed during initial motion "
                f"(ratio={last_green_ratio:.3f}, "
                f"new cells={last_green_new_cells}, "
                f"depth points={last_green_depth_points})",
                current_time,
            )
            continue

    # Phase-aware emergency guard.
    #
    # A previous version used min(all four IR sensors) during a turn.
    # A side/rear reading of 0.139 m stopped the robot even though the
    # 90-degree turn was nearly complete. Here we:
    #   1. use only front IR while driving;
    #   2. use only the two corners that sweep outward while turning;
    #   3. require three consecutive close readings.
    if phase in ("DRIVE_1", "TURN", "DRIVE_2"):
        if phase == "TURN":
            turn_sensor_name, emergency_ir_value = turn_sweep_ir(
                ir_values,
                turn_direction,
            )
            emergency_sensor_name = turn_sensor_name
        else:
            emergency_ir_value = front_ir
            emergency_sensor_name = (
                "fl" if ir_values[0] <= ir_values[1] else "fr"
            )

        if emergency_ir_value < EMERGENCY_IR_DISTANCE:
            emergency_ir_confirmation_count += 1
        else:
            emergency_ir_confirmation_count = 0

        if (
            emergency_ir_confirmation_count
            >= EMERGENCY_CONFIRMATIONS_REQUIRED
        ):
            if phase == "TURN":
                print(
                    "Turn ended early after confirmed close clearance | "
                    f"{emergency_sensor_name}="
                    f"{emergency_ir_value:.3f} m"
                )
                plan_for_current_mission(
                    "initial turn ended early by safety guard",
                    current_time,
                )
            else:
                finish_run(
                    "confirmed front IR emergency "
                    f"({emergency_sensor_name}="
                    f"{emergency_ir_value:.3f} m)"
                )
            continue

    # --------------------------------------------------------
    # Depth-based trapped-under-low-wall watchdog
    # --------------------------------------------------------

    low_wall_forward_commanded = (
        last_command_left > 0.20
        and last_command_right > 0.20
    )

    if (
        phase == "FOLLOW_PATH"
        and low_wall_forward_commanded
        and last_clearance_classification == "BLOCKED_LOW_WALL"
        and math.isfinite(last_clearance_nearest_depth)
    ):
        if not low_wall_stuck_active:
            low_wall_stuck_active = True
            low_wall_stuck_anchor_depth = last_clearance_nearest_depth
            low_wall_stuck_anchor_time = current_time

        elif (
            abs(
                last_clearance_nearest_depth
                - low_wall_stuck_anchor_depth
            )
            >= LOW_WALL_DEPTH_CHANGE_MIN
        ):
            # The relative depth changed enough, so the robot is still moving.
            low_wall_stuck_anchor_depth = last_clearance_nearest_depth
            low_wall_stuck_anchor_time = current_time

        elif (
            current_time - low_wall_stuck_anchor_time
            >= LOW_WALL_STUCK_TIMEOUT
        ):
            low_wall_stuck_active = False
            enter_collision_recovery(
                "robot appears trapped beneath a confirmed low floating wall "
                f"(depth stayed near "
                f"{last_clearance_nearest_depth:.2f} m for "
                f"{current_time - low_wall_stuck_anchor_time:.2f} s)",
                current_time,
                ir_values,
            )
            continue
    else:
        low_wall_stuck_active = False
        low_wall_stuck_anchor_depth = float("inf")
        low_wall_stuck_anchor_time = current_time

    # --------------------------------------------------------
    # No-progress watchdog
    # --------------------------------------------------------

    if phase == "FOLLOW_PATH":
        forward_commanded = (
            last_command_left >= STUCK_FORWARD_COMMAND
            and last_command_right >= STUCK_FORWARD_COMMAND
            and abs(last_command_left - last_command_right)
            <= STUCK_MAX_WHEEL_DIFFERENCE
        )

        progress_distance = math.hypot(
            robot_x - progress_anchor_x,
            robot_y - progress_anchor_y,
        )
        progress_heading_change = abs(
            normalize_angle(
                robot_theta - progress_anchor_heading
            )
        )

        # Turning, curved steering, or meaningful heading change is normal.
        # Reset the watchdog instead of calling it a collision.
        if (
            not forward_commanded
            or progress_heading_change > STUCK_MAX_HEADING_CHANGE
        ):
            progress_anchor_x = robot_x
            progress_anchor_y = robot_y
            progress_anchor_heading = robot_theta
            progress_anchor_time = current_time

        elif progress_distance >= STUCK_PROGRESS_DISTANCE:
            progress_anchor_x = robot_x
            progress_anchor_y = robot_y
            progress_anchor_heading = robot_theta
            progress_anchor_time = current_time

        elif (
            current_time - progress_anchor_time
            >= STUCK_PROGRESS_TIMEOUT
        ):
            close_obstacle_confirmed = (
                front_ir <= STUCK_OBSTACLE_CONFIRM_DISTANCE
                or last_front_lidar
                <= STUCK_OBSTACLE_CONFIRM_DISTANCE
                or last_clearance_front
                <= STUCK_OBSTACLE_CONFIRM_DISTANCE
            )

            if close_obstacle_confirmed:
                enter_collision_recovery(
                    "confirmed no-progress collision during straight "
                    f"forward motion for "
                    f"{current_time - progress_anchor_time:.2f} s",
                    current_time,
                    ir_values,
                )
                continue

            # No close obstacle evidence: do not invent a wall.
            print(
                "No-progress warning ignored because no close obstacle "
                f"was confirmed | IR={front_ir:.2f} m | "
                f"LiDAR={format_distance(last_front_lidar)} m | "
                f"clearance={format_distance(last_clearance_front)} m"
            )
            progress_anchor_x = robot_x
            progress_anchor_y = robot_y
            progress_anchor_heading = robot_theta
            progress_anchor_time = current_time
    else:
        progress_anchor_x = robot_x
        progress_anchor_y = robot_y
        progress_anchor_heading = robot_theta
        progress_anchor_time = current_time

    if phase == "GREEN_OBSERVE":
        stop_robot()

        if current_time - phase_entry_time >= GREEN_OBSERVE_DURATION:
            request_replan(
                pending_green_reason
                + " | observation complete | "
                + f"forbidden cells="
                + str(len(occupancy_grid.forbidden_cells)),
                current_time,
            )

        continue

    if phase == "VERIFY_TARGET":
        stop_robot()

        target_colour = required_target_colour()
        visual_close = target_visually_reached(
            current_target_detection,
            target_colour,
        )

        if target_check_updated:
            if visual_close:
                target_reached_confirmation_count += 1
            else:
                target_reached_confirmation_count = 0

            if (
                current_time - last_target_verify_log_time
                >= TARGET_VERIFY_LOG_INTERVAL
            ):
                last_target_verify_log_time = current_time
                print(
                    "VERIFY_TARGET | "
                    + format_target_ratio_console_line(target_colour)
                    + " | "
                    + f"reach_confirm="
                    + f"{target_reached_confirmation_count}/"
                    + f"{TARGET_REACHED_CONFIRMATIONS_REQUIRED}"
                )

        if (
            current_time - target_verify_start_time
            >= TARGET_VERIFY_DURATION
            and target_reached_confirmation_count
            >= TARGET_REACHED_CONFIRMATIONS_REQUIRED
        ):
            handle_target_reached(current_time)
            continue

        if (
            current_time - target_verify_start_time
            >= TARGET_VERIFY_TIMEOUT
        ):
            request_replan(
                "visual colour-ratio verification failed; approach again",
                current_time,
            )
            continue

        continue

    if phase == "REPLAN":
        stop_robot()

        if current_time - phase_entry_time >= REPLAN_DELAY:
            plan_for_current_mission(
                last_replan_reason,
                current_time,
            )

        continue

    if current_time - phase_entry_time < TRANSITION_PAUSE:
        stop_robot()
        continue

    # --------------------------------------------------------
    # State machine
    # --------------------------------------------------------

    if phase == "DRIVE_1":
        obstacle_ahead = (
            front_ir < FRONT_IR_STOP_DISTANCE
            or last_front_lidar < FRONT_LIDAR_STOP_DISTANCE
            or last_clearance_front < CLEARANCE_FRONT_STOP_DISTANCE
        )

        if segment_distance() >= FIRST_FORWARD_DISTANCE or obstacle_ahead:
            reason = "distance reached" if not obstacle_ahead else "obstacle ahead"
            print(
                f"First forward segment ended: {reason} | "
                f"distance={segment_distance():.3f} m"
            )
            enter_phase("TURN", current_time)
            continue

        heading_error = normalize_angle(desired_heading - robot_theta)
        correction = clamp(
            HEADING_KP * heading_error,
            -MAX_HEADING_CORRECTION,
            MAX_HEADING_CORRECTION,
        )

        set_wheel_speeds(
            FORWARD_SPEED - correction,
            FORWARD_SPEED + correction,
        )

    elif phase == "TURN":
        turned_angle = abs(
            normalize_angle(robot_theta - turn_start_heading)
        )

        if turned_angle >= TURN_ANGLE:
            print(
                f"Turn completed: "
                f"{math.degrees(turned_angle):.2f} degrees"
            )
            enter_phase("DRIVE_2", current_time)
            continue

        if turn_direction == 1:
            set_wheel_speeds(-TURN_SPEED, TURN_SPEED)
        else:
            set_wheel_speeds(TURN_SPEED, -TURN_SPEED)

    elif phase == "DRIVE_2":
        obstacle_ahead = (
            front_ir < FRONT_IR_STOP_DISTANCE
            or last_front_lidar < FRONT_LIDAR_STOP_DISTANCE
            or last_clearance_front < CLEARANCE_FRONT_STOP_DISTANCE
        )

        if segment_distance() >= SECOND_FORWARD_DISTANCE:
            plan_for_current_mission(
                "planned L-shaped mapping motion completed",
                current_time,
            )
            continue

        if obstacle_ahead:
            plan_for_current_mission(
                "second mapping segment stopped by obstacle",
                current_time,
            )
            continue

        heading_error = normalize_angle(desired_heading - robot_theta)
        correction = clamp(
            HEADING_KP * heading_error,
            -MAX_HEADING_CORRECTION,
            MAX_HEADING_CORRECTION,
        )

        set_wheel_speeds(
            FORWARD_SPEED - correction,
            FORWARD_SPEED + correction,
        )

    elif phase == "FOLLOW_PATH":
        follow_planned_path(
            current_time,
            front_ir,
            ir_values,
            green_check_updated,
        )

    # --------------------------------------------------------
    # Status output
    # --------------------------------------------------------

    if current_time - last_status_time >= STATUS_PRINT_INTERVAL:
        last_status_time = current_time

        print(
            f"phase={phase:7s} | "
            f"pose=({robot_x:.3f}, {robot_y:.3f}, "
            f"{math.degrees(robot_theta):.1f} deg) | "
            f"segment={segment_distance():.3f} m | "
            f"mission={mission_state} | "
            f"plan={active_plan_type} | "
            f"frontiers={frontiers_reached} | "
            f"waypoint={waypoint_index}/{len(planned_waypoints)} | "
            f"front IR={front_ir:.2f} m | "
            f"closest IR={closest_ir_name}:{closest_ir:.2f} m | "
            f"front LiDAR={format_distance(last_front_lidar)} m | "
            f"green={last_green_ratio:.3f} | "
            f"green cells={len(occupancy_grid.forbidden_cells)} | "
            f"clearance={format_distance(last_clearance_front)} m | "
            f"clearance class={last_clearance_classification} | "
            f"clearance cells="
            f"{len(occupancy_grid.clearance_cells)} | "
            f"target depth="
            f"{format_distance(tracked_target_depth)}"
        )
