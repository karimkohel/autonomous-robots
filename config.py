"""Configuration constants for the RosBot maze mission.

This file is a direct extraction from the provided working monolithic
controller. Values are intentionally unchanged.
"""

import math

# ============================================================
# Motion configuration: same short test that already succeeded
# ============================================================

WHEEL_RADIUS = 0.043
STARTUP_DELAY = 1.0
TRANSITION_PAUSE = 0.25

FIRST_FORWARD_DISTANCE = 0.25
SECOND_FORWARD_DISTANCE = 0.25
TURN_ANGLE = math.radians(90.0)

# Use a moderate speed while calibrating low floating-wall detection.
# The final command is still capped to the Webots motor maximum in
# set_wheel_speeds().
SPEED_MULTIPLIER = 5.0

FORWARD_SPEED = 1.8 * SPEED_MULTIPLIER
TURN_SPEED = 1.2 * SPEED_MULTIPLIER

# These values produce wheel-speed corrections, so they are scaled too in
# order to preserve approximately the same steering authority at higher speed.
HEADING_KP = 1.8 * SPEED_MULTIPLIER
MAX_HEADING_CORRECTION = 0.55 * SPEED_MULTIPLIER

# Safety thresholds.
FRONT_IR_STOP_DISTANCE = 0.14
FRONT_LIDAR_STOP_DISTANCE = 0.22
EMERGENCY_IR_DISTANCE = 0.045
EMERGENCY_CONFIRMATIONS_REQUIRED = 4

# RGB-D green-floor detection and mapping.
# RGB and depth are both 640x480 on the RosBot Astra sensor. The depth image
# supplies forward depth for green pixels, allowing them to be projected into
# the global occupancy grid and treated as forbidden space by A*.
GREEN_CHECK_INTERVAL = 0.20
GREEN_RATIO_REPLAN = 0.08
GREEN_RATIO_MAP_MINIMUM = 0.008
GREEN_SAMPLE_STEP = 6
GREEN_CONFIRMATIONS_REQUIRED = 2
GREEN_OBSERVE_DURATION = 1.20
GREEN_REARM_RATIO = 0.035
GREEN_REARM_CONFIRMATIONS_REQUIRED = 3
# Reduced semantic footprint: use the measured green cell itself and
# only a 10 cm planning margin at 5 cm/cell.
GREEN_CORE_RADIUS_CELLS = 0
GREEN_INFLATION_RADIUS_CELLS = 2
GREEN_MAX_DEPTH = 3.0
GREEN_FALLBACK_NEAR = 0.65
GREEN_FALLBACK_FAR = 0.95
GREEN_FALLBACK_HALF_WIDTH = 0.22

# RGB-D low-clearance / floating-wall detection.
#
# Test 2 showed that the most reliable feature is the normalized bottom edge
# of the nearest central surface:
#   y_max <= 0.44 -> high/passable
#   y_max >= 0.50 -> low/blocked
#   otherwise     -> uncertain
#
# The detector samples quickly enough to classify the wall before the robot
# enters underneath it.
CLEARANCE_CHECK_INTERVAL = 0.10
CLEARANCE_SAMPLE_STEP = 6

CLEARANCE_ROI_X_START = 0.40
CLEARANCE_ROI_X_END = 0.60
CLEARANCE_ROI_Y_START = 0.08
CLEARANCE_ROI_Y_END = 0.60

CLEARANCE_MAP_MIN_DEPTH = 0.20
CLEARANCE_MAP_MAX_DEPTH = 1.60
CLEARANCE_NEAREST_SURFACE_MARGIN = 0.12
CLEARANCE_MIN_VALID_POINTS = 14
CLEARANCE_MIN_SURFACE_WIDTH_PIXELS = 20

CLEARANCE_PASSABLE_BOTTOM_EDGE = 0.44
CLEARANCE_BLOCKED_BOTTOM_EDGE = 0.50
CLEARANCE_CONFIRMATION_HITS = 2

CLEARANCE_FRONT_STOP_DISTANCE = 0.45
CLEARANCE_FRONT_REPLAN_DISTANCE = 0.80
CLEARANCE_UNCERTAIN_SPEED_SCALE = 0.35

CLEARANCE_CORE_RADIUS_CELLS = 1
CLEARANCE_INFLATION_RADIUS_CELLS = 3
CLEARANCE_SKIP_LIDAR_RADIUS_CELLS = 1

# Depth-based trapped-under-wall watchdog. Encoder odometry can report motion
# while the wheels spin against a low crossbar, so also watch the depth change.
LOW_WALL_STUCK_TIMEOUT = 0.80
LOW_WALL_DEPTH_CHANGE_MIN = 0.03

# Replanning hysteresis: stop re-planning for every 1-2 noisy semantic cells.
SEMANTIC_REPLAN_COOLDOWN = 2.50
SEMANTIC_BLOCK_CONFIRMATIONS_REQUIRED = 2
SEMANTIC_SAME_REGION_RADIUS_CELLS = 4

# If a visible target has no reachable approach yet, keep exploring for a
# meaningful distance/time before trying the same target plan again.
TARGET_FAILED_RETRY_DELAY = 8.0
TARGET_FAILED_RETRY_MOVE = 0.35

# Console throttling keeps Webots from slowing down because of thousands of
# repeated detection messages.
TARGET_LOG_INTERVAL = 1.0

# During VERIFY_TARGET, print the measured ratio frequently enough to see
# exactly why the target is or is not being marked as reached.
TARGET_VERIFY_LOG_INTERVAL = 0.40

CLEARANCE_LOG_INTERVAL = 1.5

# Device offsets in the RosBot frame. The RosBot camera slot is at x=-0.027 m;
# the Astra depth sensor is at x=+0.027 m inside the camera body, so its net
# x offset is approximately zero. Its lateral offset is +0.037 m.
DEPTH_CAMERA_OFFSET_X = 0.0
DEPTH_CAMERA_OFFSET_Y = 0.037

# Path-following configuration.
WAYPOINT_REACHED_DISTANCE = 0.07
GOAL_REACHED_DISTANCE = 0.10
PATH_FORWARD_SPEED = 1.80 * SPEED_MULTIPLIER
PATH_TURN_SPEED = 1.10 * SPEED_MULTIPLIER
PATH_HEADING_KP = 2.2 * SPEED_MULTIPLIER
PATH_MAX_HEADING_CORRECTION = 0.75 * SPEED_MULTIPLIER
TURN_IN_PLACE_THRESHOLD = math.radians(18.0)
FORWARD_ALLOWED_HEADING_ERROR = math.radians(28.0)

# Collision recovery.
#
# Replanning alone cannot free a robot that is already touching a wall.
# The controller therefore backs along the recently travelled route, turns
# away from the closer front corner, marks the contact region as blocked,
# and only then asks A* for a new path.
# Match the normal stopping boundary so there is no dead zone where
# the robot stops and replans forever without entering physical recovery.
CONTACT_RECOVERY_IR_DISTANCE = 0.115
CONTACT_RECOVERY_LIDAR_DISTANCE = 0.20
RECOVERY_BACKUP_DISTANCE = 0.13
RECOVERY_BACKUP_SPEED = 1.10 * SPEED_MULTIPLIER
RECOVERY_REAR_STOP_DISTANCE = 0.055
RECOVERY_BACKUP_TIMEOUT = 4.50
RECOVERY_TURN_ANGLE = math.radians(25.0)
RECOVERY_TURN_SPEED = 0.90 * SPEED_MULTIPLIER
RECOVERY_TURN_TIMEOUT = 2.50

# Dead-zone escape guard. If the robot repeatedly stops from the same pose
# because a tight slanted corridor looks barely too close, do not replan
# forever. After a few repeated near-obstacle replans, force a controlled
# short physical recovery.
REPEATED_OBSTACLE_RECOVERY_ENABLED = True
REPEATED_OBSTACLE_WINDOW = 8.0
REPEATED_OBSTACLE_RADIUS = 0.16
REPEATED_OBSTACLE_COUNT = 3
REPEATED_OBSTACLE_MIN_IR = 0.115
REPEATED_OBSTACLE_MAX_IR = 0.18
RECOVERY_CONTACT_MARK_RADIUS_CELLS = 0
RECOVERY_CONTACT_MARK_MIN_DISTANCE = 0.18
RECOVERY_CONTACT_MARK_MAX_DISTANCE = 0.34

# Recovery marks are temporary. They prevent A* from immediately selecting the
# same collision point without permanently sealing a corridor.
TEMP_RECOVERY_TTL = 2.0
TEMP_RECOVERY_INFLATION_RADIUS_CELLS = 0

# Competition-focused simplification for Maze 3/4/5.
# The solved/target mazes do not need low/floating-wall avoidance, and the
# RGB-D low-clearance layer can over-block tight zigzags. Keep measuring and
# visualizing it, but do not let it block A* or stop path following.
LOW_CLEARANCE_BLOCKS_PLANNING = False
LOW_CLEARANCE_TRIGGERS_REPLAN = False
LOW_CLEARANCE_STOPS_MOTION = False
LOW_CLEARANCE_SLOWS_MOTION = False
LOW_CLEARANCE_RECORD_CELLS = False
DRAW_LOW_CLEARANCE_CELLS = False

# Narrow-corridor execution tuning. These do not change global A* logic;
# they only make the local follower less jumpy in tight zigzags.
NARROW_CORRIDOR_SIDE_DISTANCE = 0.30
NARROW_CORRIDOR_FRONT_LIDAR_DISTANCE = 0.55
NARROW_CORRIDOR_SPEED_SCALE = 0.55
NARROW_CORRIDOR_TURN_SPEED_SCALE = 0.45

# Proven-traversable corridor. If the robot physically drives through a place,
# the inflated safety margin and temporary recovery marks around that centre
# path are softened later. Raw LiDAR occupied cells and green forbidden cells
# remain hard blocked.
TRAVERSED_CORRIDOR_RADIUS_CELLS = 3

# Aggressive narrow mode is used only after repeated local failures. It keeps
# raw walls and green blocked, but ignores normal wall inflation and temporary
# recovery marks so the robot can try tight zigzags slowly.
NARROW_MODE_ENABLED = True
NARROW_MODE_CLEAR_TEMPORARY_RECOVERY = True

# Detect wheel motion without meaningful robot translation.
# The watchdog is active only for near-straight forward commands. Turning in
# place must never be classified as being stuck.
STUCK_PROGRESS_DISTANCE = 0.025
STUCK_PROGRESS_TIMEOUT = 1.50
STUCK_FORWARD_COMMAND = 0.45 * SPEED_MULTIPLIER
STUCK_MAX_WHEEL_DIFFERENCE = 0.28 * SPEED_MULTIPLIER
STUCK_MAX_HEADING_CHANGE = math.radians(4.0)
STUCK_OBSTACLE_CONFIRM_DISTANCE = 0.22

# No-reachable-frontier fallback.
# First remove temporary recovery marks, then rotate once to refresh the map.
NO_FRONTIER_SCAN_SPEED = 0.70 * SPEED_MULTIPLIER
NO_FRONTIER_SCAN_TARGET = math.radians(350.0)
NO_FRONTIER_SCAN_TIMEOUT = 13.0
MAX_CONSECUTIVE_NO_FRONTIER_SCANS = 2

# If A* sees no reachable frontier after scans, do not terminate immediately.
# Move a short safe distance to collect new LiDAR/RGB-D evidence, then replan.
# This keeps the Maze 3 architecture but prevents Maze 4/5 from ending early
# when the local map is temporarily closed.
SAFE_EXPLORE_SPEED = 1.00 * SPEED_MULTIPLIER
SAFE_EXPLORE_TURN_SPEED = 0.70 * SPEED_MULTIPLIER
SAFE_EXPLORE_DISTANCE = 0.40
SAFE_EXPLORE_TIMEOUT = 5.50
SAFE_EXPLORE_INITIAL_TURN = 0.85
SAFE_EXPLORE_GREEN_TURN_RATIO = 0.055
SAFE_EXPLORE_OBSTACLE_DISTANCE = 0.23
MAX_NO_FRONTIER_SAFE_EXPLORES = 12

# Continuous exploration configuration.
# Frontier exploration continues until the blue and yellow mission is complete.
MAX_FRONTIERS_TO_VISIT = 999
REPLAN_DELAY = 0.25
MAX_TOTAL_PLANNING_ATTEMPTS = 250

# Blue/yellow RGB-D detection.
TARGET_CHECK_INTERVAL = 0.20
TARGET_SAMPLE_STEP = 5
TARGET_MIN_SAMPLES = 10
TARGET_CONFIRMATIONS_REQUIRED = 2
TARGET_TRACK_GATE = 0.55
TARGET_TRACK_ALPHA = 0.35
TARGET_MAX_DEPTH = 5.0
TARGET_REPLAN_COOLDOWN = 2.50
TARGET_CACHE_MAX_AGE = 180.0

# Cached-target improvement:
# A yellow position observed on the way to blue can be reused immediately
# after blue is reached. Yellow is visually distinctive, so one reliable
# RGB-D position is enough to start a target A* plan; the final arrival is
# still verified visually near the pillar.
YELLOW_CACHE_CONFIRMATIONS_AFTER_BLUE = 1

# Run target detection even while rotating in fallback scans. This is what
# lets the robot stop scanning as soon as it sees yellow.
TARGET_SCAN_CACHE_ENABLED = True

# Safe approach planning around a pillar.
TARGET_APPROACH_MIN_RADIUS = 0.30
TARGET_APPROACH_MAX_RADIUS = 0.45
TARGET_PREFERRED_STANDOFF = 0.35
TARGET_APPROACH_RADIUS_STEP = 0.05
TARGET_APPROACH_ANGLE_STEP = math.radians(15.0)
TARGET_LINE_OF_SIGHT_MARGIN_CELLS = 6

# Reaching/verification.
TARGET_VERIFY_DURATION = 0.60
TARGET_VERIFY_TIMEOUT = 2.50
# Visual colour-ratio arrival. Depth and cached global distance are no
# longer used to decide whether a pillar has been reached.
#
# Calibrated starting values:
# - Blue is considered reached when it fills at least 24% of sampled pixels.
# - Yellow is narrower, so its reached ratio is lower.
BLUE_DETECTION_MIN_RATIO = 0.006
YELLOW_DETECTION_MIN_RATIO = 0.0025
BLUE_REACHED_COLOR_RATIO = 0.24
YELLOW_REACHED_COLOR_RATIO = 0.065
TARGET_REACHED_MIN_BBOX_HEIGHT_RATIO = 0.75
TARGET_REACHED_CONFIRMATIONS_REQUIRED = 1


# ============================================================
# Mapping configuration
# ============================================================

MAP_RESOLUTION = 0.05       # 5 cm per cell
GRID_SIZE = 240             # 12 m x 12 m
MIN_MAPPING_DISTANCE = 0.24
MAX_MAPPING_DISTANCE = 5.5

SCAN_INTERVAL = 0.10
MAP_SAVE_INTERVAL = 1.0
STATUS_PRINT_INTERVAL = 0.50

LIDAR_OFFSET_X = 0.02
LIDAR_OFFSET_Y = 0.00

# Evidence values: zero means unknown.
FREE_UPDATE = -1
OCCUPIED_UPDATE = 3
MIN_EVIDENCE = -12
MAX_EVIDENCE = 12
FREE_THRESHOLD = -2
OCCUPIED_THRESHOLD = 3

# Point-cloud sectors used by the safety layer.
FRONT_HALF_ANGLE = math.radians(25.0)
LEFT_MIN_ANGLE = math.radians(45.0)
LEFT_MAX_ANGLE = math.radians(135.0)
RIGHT_MIN_ANGLE = math.radians(-135.0)
RIGHT_MAX_ANGLE = math.radians(-45.0)


# ============================================================
# Planning configuration
# ============================================================

# 3 cells x 5 cm = 15 cm LiDAR inflation around raw wall hits.
# Exact pre-breadcrumb working baseline: normal wall inflation, with the
# aggressive traversed-corridor softening from the last run that reached blue.
# Raw occupied wall cells and green remain hard blocked.
INFLATION_RADIUS_CELLS = 3

# LiDAR rays deliberately skip cells occupied by the robot body, so these
# cells can remain unknown. During planning only, treat this small area around
# the current robot pose as free so A* can leave the start position.
ROBOT_START_CLEARANCE_CELLS = 3

# A frontier cluster smaller than this is treated as sensor noise.
MIN_FRONTIER_CLUSTER_SIZE = 5

# Avoid selecting a frontier immediately beside the robot.
MIN_FRONTIER_DISTANCE_CELLS = 5

# Limit how many clusters receive an A* attempt.
MAX_FRONTIER_CLUSTERS_TO_TEST = 30

# A small information-gain reward: large frontier clusters are preferred
# when path lengths are similar.
FRONTIER_SIZE_REWARD = 0.12

# A* uses 8-connected movement.
SQRT_2 = math.sqrt(2.0)



# ============================================================
# Robot physical footprint and sticky-escape planning
# ============================================================
# RosBot PROTO-derived dimensions used by the planner. The body box is about
# 0.20 m x 0.15 m, wheel centres are at y=+-0.110 m, and the wheels add a
# little width. A circular planning footprint of about 15 cm radius preserves
# the successful old A* behaviour while making the assumption explicit.
ROBOT_BODY_LENGTH = 0.200
ROBOT_BODY_WIDTH = 0.150
ROBOT_WHEEL_TRACK = 0.220
ROBOT_ESTIMATED_TOTAL_WIDTH = 0.260
ROBOT_ESTIMATED_TOTAL_LENGTH = 0.240
ROBOT_PLANNING_SAFETY_MARGIN = 0.020
ROBOT_PLANNING_RADIUS_METRES = (
    ROBOT_ESTIMATED_TOTAL_WIDTH / 2.0 + ROBOT_PLANNING_SAFETY_MARGIN
)
ROBOT_PLANNING_RADIUS_CELLS = max(
    INFLATION_RADIUS_CELLS,
    math.ceil(ROBOT_PLANNING_RADIUS_METRES / MAP_RESOLUTION),
)

# Third planning mode: sticky escape.
# Triggered when the robot keeps failing to find target/frontier plans from
# almost the same pose. It selects a nearby open free-space region, navigates
# there, then returns to normal target/frontier planning.
STICKY_FAILURE_WINDOW = 35.0
STICKY_FAILURE_RADIUS = 0.35
STICKY_FAILURE_COUNT = 3
MAX_STICKY_ESCAPES = 10

ESCAPE_MIN_DISTANCE = 0.35
ESCAPE_MAX_DISTANCE = 1.35
ESCAPE_PREFERRED_DISTANCE = 0.75
ESCAPE_CANDIDATE_STRIDE_CELLS = 2
ESCAPE_OPEN_REGION_RADIUS_CELLS = max(5, ROBOT_PLANNING_RADIUS_CELLS + 3)
ESCAPE_MIN_FREE_CELLS = 35
ESCAPE_MAX_CANDIDATES_TO_TEST = 70
ESCAPE_UNKNOWN_EDGE_REWARD = 0.08
ESCAPE_OPEN_REGION_REWARD = 0.045
ESCAPE_TARGET_DISTANCE_WEIGHT = 0.35
ESCAPE_DISTANCE_PREFERENCE_WEIGHT = 0.18
